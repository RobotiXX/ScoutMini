#!/usr/bin/env bash
set -euo pipefail

output_dir="${1:?usage: $0 OUTPUT_DIRECTORY}"
base_url='https://drive.usercontent.google.com/download?export=download&confirm=t&id='

download_clip() {
  local name="$1"
  local db_id="$2"
  local metadata_id="$3"
  local expected_size="$4"
  local clip_dir="${output_dir}/${name}"
  local db_path="${clip_dir}/${name}_merged.db3"

  mkdir -p "${clip_dir}"
  wget -c -O "${clip_dir}/metadata.yaml" "${base_url}${metadata_id}"
  wget -c -O "${db_path}" "${base_url}${db_id}"

  local actual_size
  actual_size="$(stat --format='%s' "${db_path}")"
  if [[ "${actual_size}" != "${expected_size}" ]]; then
    echo "Size mismatch for ${db_path}: ${actual_size}" >&2
    exit 1
  fi
}

download_clip \
  engr_elev_1_1 \
  1wgC0Q-T8yUOvezoYrWU-EkELtEvTdcO4 \
  1jevehoq6V5XF_KxXslwndV6Ao3mgiiPg \
  594903040
download_clip \
  engr_hall_1_1 \
  1nb2dZnbHudJ9Z5NypMdtWkoOclu9b62a \
  1RiFffOnfkFHG_4C3R0c8dWajduOzUZFR \
  538234880
download_clip \
  innovation_1_1 \
  1Gv17nk0xJCVyS04d45ANshVdPm1fvdeo \
  1eiwJe_MtKkoiwam8PDLeXEAR0EyiadMT \
  959131648
download_clip \
  jc_1_1 \
  1trLbxn_XqCTLfyR80MuNeveeXvh9uset \
  1aX0ZAJVirzD2aXeZDo8VEjfNlim5Kh9c \
  397131776

echo "ACME GMU evaluation suite ready in ${output_dir}"
