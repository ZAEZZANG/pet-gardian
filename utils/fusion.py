"""
fusion.py
- mmWave + Thermal 센서 융합 로직
- "픽셀 수만 보면 거리 때문에 사람/반려동물 구분이 꼬이는 문제"를
  mmWave 거리/크기/호흡 정보로 보완하는 역할

가정:
- mmwave_data: dict 형태
    {
        "bpm": float or int,      # 호흡수 (0이면 없음/측정불가)
        "motion": float or int,   # 움직임 레벨 (mmWave가 주는 값)
        "distance": float or None,# 센서와 대상 거리 (m 단위 가정)
        "size": float or int      # 반사 강도/크기 등 (값 범위는 모듈에 따라 튜닝)
    }

- thermal 측 값:
    temp: float                  # 핫 영역 평균 온도
    hot_pixels: int              # 핫 픽셀 개수
    thermal_entity: str          # "none" / "pet" / "human_or_large" / "unknown"
"""

from dataclasses import dataclass

# ==========================
# CONFIG (실험하면서 튜닝해야 하는 값들)
# ==========================

NEAR_RANGE_M = 1.5          # 이 거리 이내면 "가까운 대상"으로 간주
FAR_IGNORE_RANGE_M = 4.0    # 이 거리 넘으면 웬만하면 무시 (너네 설치 환경 보고 조정)

# mmWave 기준
BPM_MIN_VALID = 5           # 이보다 작은 bpm은 노이즈/미측정으로 취급
BPM_MAX_VALID = 80          # 이보다 큰 bpm도 비정상으로 취급

HUMAN_SIZE_THRESHOLD = 40   # mmWave size가 이 이상이면 "큰 대상" (사람/큰 물체) 후보
PET_SIZE_MAX = 40           # 이 이하일 때 소형 대상(반려동물) 쪽으로 봄 (모듈 기준값에 맞춰 수정)


@dataclass
class FusionResult:
    entity: str        # "pet" / "human" / "none" / "unknown"
    confidence: float  # 0.0 ~ 1.0 (대략적인 신뢰도)
    reason: str        # 어떤 근거로 그렇게 판단했는지 설명 텍스트


# ==========================
# mmWave 측 단일 분류 (거리/크기/호흡만 보고 1차 분류)
# ==========================

def classify_mmwave(mm_data: dict) -> str:
    """
    mmWave 데이터만 보고 대략적인 대상 분류:
    Returns:
        "no_target"     : 호흡 X + 거리 None or 너무 멂
        "near_small"    : 가까이 있는 작은 대상
        "near_large"    : 가까이 있는 큰 대상(사람/큰 물체)
        "far_target"    : 멀리 있는 대상 (우선순위 낮음)
    """
    bpm = mm_data.get("bpm", 0) or 0
    dist = mm_data.get("distance", None)
    size = mm_data.get("size", 0) or 0

    has_life = BPM_MIN_VALID <= bpm <= BPM_MAX_VALID

    if dist is None:
        # 거리 정보 없음 → 호흡도 없으면 대상 X로 간주
        return "no_target" if not has_life else "far_target"

    if dist > FAR_IGNORE_RANGE_M:
        # 너무 멀면 거의 무시
        return "no_target"

    # 가까운/먼 기준 나누기
    if dist <= NEAR_RANGE_M:
        # 가까운 대상
        if size >= HUMAN_SIZE_THRESHOLD:
            return "near_large"
        else:
            return "near_small"
    else:
        # 멀리 있는 대상
        return "far_target"


# ==========================
# Fusion 메인 함수
# ==========================

def fuse_sensors(mm_data: dict,
                 temp: float,
                 hot_pixels: int,
                 thermal_entity: str) -> FusionResult:
    """
    mmWave + Thermal 정보를 동시에 보고 최종 대상 분류.

    Args:
        mm_data: mmWave dict (bpm, motion, distance, size 포함)
        temp: thermal 핫 영역 평균 온도
        hot_pixels: thermal 핫 픽셀 개수
        thermal_entity: thermal.classify_thermal_entity(hot_pixels)의 결과

    Returns:
        FusionResult(entity, confidence, reason)
    """
    bpm = mm_data.get("bpm", 0) or 0
    motion = mm_data.get("motion", 0) or 0
    dist = mm_data.get("distance", None)
    size = mm_data.get("size", 0) or 0

    mm_class = classify_mmwave(mm_data)
    has_life = (BPM_MIN_VALID <= bpm <= BPM_MAX_VALID)

    # 1️⃣ 생체 신호 자체가 없는 경우 (bpm 유효 값 X)
    if not has_life:
        # thermal에서도 따뜻한 물체 거의 없으면 → none
        if thermal_entity == "none" or hot_pixels == 0:
            return FusionResult(
                entity="none",
                confidence=0.9,
                reason="mmWave 호흡 없음 + thermal 따뜻한 대상 거의 없음"
            )
        # 따뜻한 물체는 있는데 호흡은 없음 → 뜨거운 물체(난방기, 노트북 등)일 가능성
        else:
            return FusionResult(
                entity="unknown",
                confidence=0.5,
                reason=f"thermal에 따뜻한 물체는 있으나(both: entity={thermal_entity}, hot_pixels={hot_pixels}), "
                       f"mmWave 호흡 신호는 없어 생체 여부 애매함"
            )

    # 2️⃣ 가까운 대상이 있고, thermal에서도 뭔가 보이는 경우
    if mm_class in ("near_small", "near_large"):
        # 가까이 있는 대상 + thermal에서 pet 크기
        if mm_class == "near_small" and thermal_entity == "pet":
            return FusionResult(
                entity="pet",
                confidence=0.9,
                reason=f"가까운 작은 대상(size={size}, dist={dist:.2f}m) + thermal pet 패턴(hot_pixels={hot_pixels})"
            )

        # 가까운 큰 대상 + thermal에서도 큰 덩어리
        if mm_class == "near_large" and thermal_entity == "human_or_large":
            return FusionResult(
                entity="human",
                confidence=0.9,
                reason=f"가까운 큰 대상(size={size}, dist={dist:.2f}m) + thermal large 패턴(hot_pixels={hot_pixels})"
            )

        # 조합이 조금 어긋나는 경우들 (예: mm는 small인데 thermal은 human_or_large라거나)
        # → 그래도 반려동물 쪽/사람 쪽으로 약하게 기울여서 판단
        if mm_class == "near_small" and thermal_entity == "human_or_large":
            return FusionResult(
                entity="pet",
                confidence=0.6,
                reason=f"mmWave는 작은 대상(size={size})을 가까이서 감지, "
                       f"thermal은 큰 패턴으로 보이지만 거리 효과일 수 있음"
            )

        if mm_class == "near_large" and thermal_entity == "pet":
            return FusionResult(
                entity="human",
                confidence=0.6,
                reason=f"mmWave는 큰 대상(size={size})을 가까이서 감지, "
                       f"thermal은 작게 보이지만 각도/프레임 영향일 수 있음"
            )

        # thermal이 unknown이거나 경계값 근처
        return FusionResult(
            entity="unknown",
            confidence=0.5,
            reason=f"가까운 대상 감지(mm_class={mm_class}), thermal 패턴은 애매(entity={thermal_entity}, hot_pixels={hot_pixels})"
        )

    # 3️⃣ 대상이 멀리 있을 때 (far_target)
    if mm_class == "far_target":
        # thermal에서도 pet 크기라면 → 멀리 있는 반려동물일 수도 있지만 우선순위는 낮게
        if thermal_entity == "pet":
            return FusionResult(
                entity="pet",
                confidence=0.6,
                reason=f"mmWave가 멀리 대상 감지(dist={dist:.2f}m), "
                       f"thermal은 pet 크기(hot_pixels={hot_pixels}) → 멀리 있는 반려동물 가능성"
            )
        # thermal도 human_or_large → 멀리 있는 사람/큰 물체
        if thermal_entity == "human_or_large":
            return FusionResult(
                entity="human",
                confidence=0.6,
                reason=f"mmWave가 멀리 큰 대상 감지(dist={dist:.2f}m), "
                       f"thermal도 large 패턴(hot_pixels={hot_pixels})"
            )

        # thermal에서 거의 안 보이면 → 그냥 멀리 있는 잡 신호
        return FusionResult(
            entity="none",
            confidence=0.7,
            reason=f"mmWave가 멀리 대상 감지(dist={dist:.2f}m)하나, "
                   f"thermal에서 뚜렷한 따뜻한 패턴이 없어 무시"
        )

    # 4️⃣ mm_class == "no_target" 인데 thermal은 따뜻한 물체를 본 경우
    if mm_class == "no_target":
        if thermal_entity == "pet":
            # mmWave 신호가 잘 안 잡히는 위치/각도일 수 있음 → 약한 pet 후보
            return FusionResult(
                entity="pet",
                confidence=0.5,
                reason=f"mmWave 대상 없음으로 분류되지만, thermal은 pet 크기(hot_pixels={hot_pixels})"
            )
        elif thermal_entity == "human_or_large":
            return FusionResult(
                entity="human",
                confidence=0.5,
                reason=f"mmWave 대상 없음으로 분류되지만, thermal은 large 크기(hot_pixels={hot_pixels})"
            )
        else:
            return FusionResult(
                entity="none",
                confidence=0.8,
                reason="mmWave 대상 없음 + thermal 패턴도 약함"
            )

    # 여기까지 왔다면 애매한 경우
    return FusionResult(
        entity="unknown",
        confidence=0.4,
        reason=f"mm_class={mm_class}, thermal_entity={thermal_entity}, hot_pixels={hot_pixels} → 추가 튜닝 필요"
    )
