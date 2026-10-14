# Source this file to get a clean XTDrone ROS 2 Jazzy shell:
#   source scripts/xtdrone_env.sh
#
# It leaves any Python virtualenv (e.g. one activated from ~/.profile), drops
# any other ROS distribution sourced from ~/.bashrc, then sources
# ROS 2 Jazzy and the XTDrone workspace. Only the current shell is changed.

# ROS setup files are not 'set -u' safe; relax it while sourcing and restore it at the end.
case $- in *u*) _xtd_nounset=1; set +u ;; *) _xtd_nounset=0 ;; esac

# Terminals inside snap applications (e.g. the VS Code snap) export GTK/locale paths
# into the snap; the Gazebo GUI then loads incompatible libraries and crashes.
if [[ "${SNAP:-}" == /snap/* ]]; then
  unset GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR LOCPATH GDK_PIXBUF_MODULE_FILE \
        GDK_PIXBUF_MODULEDIR GSETTINGS_SCHEMA_DIR SNAP SNAP_NAME SNAP_REVISION
fi

if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  if declare -F deactivate >/dev/null; then
    deactivate
  else
    PATH="$(printf '%s' "${PATH}" | tr ':' '\n' | grep -vxF "${VIRTUAL_ENV}/bin" | paste -sd: -)"
    export PATH
    unset VIRTUAL_ENV
  fi
fi

if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "jazzy" ]]; then
  _xtd_clean() {
    printf '%s' "${!1:-}" | tr ':' '\n' | grep -v "^/opt/ros/${ROS_DISTRO}" | paste -sd: -
  }
  for _xtd_var in PATH PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH CMAKE_PREFIX_PATH; do
    _xtd_val="$(_xtd_clean "${_xtd_var}")"
    if [[ -n "${_xtd_val}" ]]; then
      export "${_xtd_var}=${_xtd_val}"
    else
      unset "${_xtd_var}"
    fi
  done
  unset ROS_PACKAGE_PATH ROS_ETC_DIR ROS_ROOT ROS_MASTER_URI ROS_VERSION ROS_PYTHON_VERSION ROS_DISTRO
  unset _xtd_var _xtd_val
  unset -f _xtd_clean
fi

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed; run scripts/setup_ubuntu_24_04.sh" >&2
  [[ ${_xtd_nounset} -eq 1 ]] && set -u
  return 1 2>/dev/null || exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

_xtd_ws="${XTDRONE_WS:-${HOME}/xtdrone_ros2_ws}"
if [[ -f "${_xtd_ws}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${_xtd_ws}/install/setup.bash"
fi
unset _xtd_ws

XTDRONE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export XTDRONE_ROOT

[[ ${_xtd_nounset} -eq 1 ]] && set -u
unset _xtd_nounset
