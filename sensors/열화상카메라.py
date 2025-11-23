def _parse_one_frame(frame: bytes):
    """
    MR60BHA1 프로토콜 기준 단일 프레임 파싱
    frame 형식:
      0: 0x53 'S'
      1: 0x59 'Y'
      2: control
      3: command
      4-5: length (big-endian)
      6.. : data (len 바이트)
      마지막-3: checksum
      마지막-2, -1: 0x54 0x43 ('T','C')
    """
    if len(frame) < 9:
        return None

    ctrl = frame[2]
    cmd = frame[3]
    data_len = (frame[4] << 8) | frame[5]
    data_start = 6
    data_end = data_start + data_len
    if data_end + 3 > len(frame):
        return None

    data = frame[data_start:data_end]
    checksum = frame[data_end]
    tail = frame[data_end + 1 : data_end + 3]

    if tail != b"\x54\x43":   # 'T','C'
        return None

    # (선택사항) 체크섬 확인: 모든 바이트 합의 하위 8비트 == checksum
    calc = sum(frame[0:data_end]) & 0xFF
    if calc != checksum:
        # print(f"[MMW] checksum mismatch: got {checksum:02X}, calc {calc:02X}")
        return None

    # ctrl/command 조합에 따라 의미 파싱
    # 0x81 0x02 : 호흡 값 (breath value)
    # 0x80 0x03 : 몸 움직임 강도 (body movement parameter)
    result = {}

    if ctrl == 0x81 and cmd == 0x02 and data_len >= 1:
        # breath value: 0~30 정도, 여기서는 bpm 느낌으로 사용
        breath_val = data[0]
        result["bpm"] = float(breath_val)

    if ctrl == 0x80 and cmd == 0x03 and data_len >= 1:
        motion_val = data[0]  # 0~100 정도
        result["motion"] = float(motion_val)

    return result or None


def get_mmwave_readings():
    """
    실제 MR60BHA1 UART 데이터에서
    - bpm (호흡수/심박수 느낌 값)
    - motion (움직임 강도)
    를 추출해서 반환.

    반환 형식:
      {"bpm": float, "motion": float}
    값이 아직 없으면 0.0 으로 대체.
    """
    global _mmw_ser, _mmw_buf, _mmw_last_bpm, _mmw_last_motion

    # UART 미초기화 시 한 번만 초기화
    if _mmw_ser is None:
        init_mmwave_uart()

    try:
        chunk = _mmw_ser.read(128)
    except Exception as e:
        print(f"[MMW] read error: {e}")
        return {
            "bpm": _mmw_last_bpm if _mmw_last_bpm is not None else 0.0,
            "motion": _mmw_last_motion,
        }

    if chunk:
        _mmw_buf += chunk

        # 버퍼에서 여러 프레임을 연속 파싱
        while True:
            start = _mmw_buf.find(b"\x53\x59")  # 'S','Y'
            if start < 0:
                # 헤더 못 찾으면 버퍼 초기화
                _mmw_buf = b""
                break

            # 최소 길이(헤더+ctrl+cmd+len+chk+tail) 9바이트
            if len(_mmw_buf) - start < 9:
                # 더 읽어야 함
                if start > 0:
                    _mmw_buf = _mmw_buf[start:]
                break

            # length 읽어서 전체 프레임 길이 계산
            if start + 6 > len(_mmw_buf):
                # len 필드까지 아직 안 들어옴
                break
            data_len = (_mmw_buf[start+4] << 8) | _mmw_buf[start+5]
            frame_len = 2 + 1 + 1 + 2 + data_len + 1 + 2  # SY + ctrl + cmd + len + data + chk + TC

            if len(_mmw_buf) - start < frame_len:
                # 프레임 전체가 아직 안 들어옴
                if start > 0:
                    _mmw_buf = _mmw_buf[start:]
                break

            frame = _mmw_buf[start:start+frame_len]
            _mmw_buf = _mmw_buf[start+frame_len:]  # 파싱한 프레임 제거

            parsed = _parse_one_frame(frame)
            if parsed:
                if "bpm" in parsed:
                    _mmw_last_bpm = parsed["bpm"]
                    # print(f"[MMW] breath={_mmw_last_bpm}")
                if "motion" in parsed:
                    _mmw_last_motion = parsed["motion"]
                    # print(f"[MMW] motion={_mmw_last_motion}")

    # 값이 아직 None이면 기본값 0.0
    bpm = _mmw_last_bpm if _mmw_last_bpm is not None else 0.0
    motion = _mmw_last_motion

    return {
        "bpm": bpm,
        "motion": motion,
    }
