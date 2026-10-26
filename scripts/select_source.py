"""Diagnostic source selection. Does not authorize robot execution."""

VALID_VARIANTS = {"VARa", "VARb", "VARc"}
VALID_STATUSES = {"matched", "rejected", "uncalibrated"}


def select_source(scene, requested_variant):
    result = {
        "requested_variant": requested_variant,
        "status": "blocked",
        "reason": None,
        "candidate_id": None,
        "matching_candidate_ids": [],
        "execution_allowed": False,
    }

    def blocked(reason):
        result["reason"] = reason
        return result

    if requested_variant not in VALID_VARIANTS:
        return blocked("unsupported_variant")

    if not isinstance(scene, dict):
        return blocked("invalid_scene")

    candidates = scene.get("source_candidates")
    if not isinstance(candidates, list):
        return blocked("invalid_candidate_list")

    seen = set()
    observations = []

    for candidate in candidates:
        if not isinstance(candidate, dict):
            return blocked("invalid_candidate")

        identifier = candidate.get("candidate_id")
        if not isinstance(identifier, str) or not identifier.strip():
            return blocked("invalid_candidate_id")
        if identifier in seen:
            return blocked("duplicate_candidate_ids")
        seen.add(identifier)

        classification = candidate.get("metric_classification")
        if not isinstance(classification, dict):
            return blocked("missing_classification")

        status = classification.get("status")
        variant = classification.get("variant")
        if status not in VALID_STATUSES:
            return blocked("invalid_classification_status")
        if status == "matched" and variant not in VALID_VARIANTS:
            return blocked("invalid_matched_variant")
        if status != "matched" and variant is not None:
            return blocked("inconsistent_classification")

        observations.append((identifier, status, variant))

    if any(status == "uncalibrated" for _, status, _ in observations):
        return blocked("uncalibrated_candidates")

    if any(status == "rejected" for _, status, _ in observations):
        return blocked("unresolved_source_candidates")

    # Large rejected image regions could also contain a missed board.
    rejected_regions = scene.get("rejected_source_regions", [])
    if not isinstance(rejected_regions, list):
        return blocked("invalid_rejected_region_list")
    if rejected_regions:
        return blocked("unresolved_source_regions")

    matches = [
        identifier
        for identifier, _, variant in observations
        if variant == requested_variant
    ]
    result["matching_candidate_ids"] = matches

    if not matches:
        return blocked("requested_variant_unavailable")
    if len(matches) != 1:
        return blocked("multiple_matching_candidates")

    result.update({
        "status": "selected_diagnostic",
        "reason": None,
        "candidate_id": matches[0],
    })
    return result
