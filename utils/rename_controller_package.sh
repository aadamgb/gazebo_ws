#!/usr/bin/env bash
#
# Rename (or clone) an MRS controller plugin package.
#
#   ./rename_controller_package.sh <src_dir> <dest_dir> <new_pkg> <NewClass>
#
# e.g.  ./rename_controller_package.sh src/example_controller_plugin src/srt_controller \
#           srt_controller ActuatorsController
#
# The old names are read out of the package itself (package.xml, plugins.xml),
# so this works on any package following the MRS controller-plugin layout.
#
# Renames, consistently:
#   - package / CMake project / ament install dirs
#   - C++ namespace, class, PLUGINLIB_EXPORT_CLASS, get_package_share_directory()
#   - the CMake library target and the <library path> in plugins.xml
#   - src/<ns>.cpp and config/<ns>.yaml, including the param key inside the yaml
#     (that key MUST equal the `namespace:` field in custom_config, or the
#     controller loads with no gains)
#   - the `address:` / `namespace:` / controller names in any custom_config.yaml
#
# Afterwards, delete the old build/ and install/ trees for the previous package
# name -- a stale install tree keeps advertising the old class through the
# pluginlib index and it stays loadable.
#
# Not touched: the <description> in package.xml / plugins.xml, and the maintainer
# and author fields. Those are free text, not derivable from a name -- edit them
# by hand.

set -euo pipefail

if [ $# -ne 4 ]; then
  sed -n '2,27p' "$0" | sed 's/^#//'
  exit 1
fi

SRC="${1%/}"; DEST="${2%/}"; NEW_PKG="$3"; NEW_CLASS="$4"

[ -f "$SRC/package.xml" ] || { echo "error: $SRC is not a ROS package"; exit 1; }
[ -f "$SRC/plugins.xml" ] || { echo "error: $SRC has no plugins.xml"; exit 1; }
[ -e "$DEST" ] && { echo "error: $DEST already exists"; exit 1; }

# --- discover the old names -------------------------------------------------

OLD_PKG=$(sed -n 's:.*<name>\(.*\)</name>.*:\1:p' "$SRC/package.xml" | head -1)
OLD_LIB=$(sed -n 's:.*<library path="\([^"]*\)".*:\1:p' "$SRC/plugins.xml" | head -1)
OLD_CLASS=$(sed -n 's#.*<class name="[^"]*::\([^"]*\)".*#\1#p' "$SRC/plugins.xml" | head -1)

# the controller's own param file is the one config/*.yaml that is not a custom_config
OLD_NS=$(find "$SRC/config" -name '*.yaml' ! -name '*custom_config*' -printf '%f\n' | head -1 | sed 's/\.yaml$//')

[ -n "$OLD_PKG" ] && [ -n "$OLD_LIB" ] && [ -n "$OLD_CLASS" ] && [ -n "$OLD_NS" ] || {
  echo "error: could not determine the old names"; exit 1; }

# --- derive the new names ---------------------------------------------------

# ActuatorsController -> actuators_controller
NEW_NS=$(echo "$NEW_CLASS" | sed 's/\([a-z0-9]\)\([A-Z]\)/\1_\2/g' | tr '[:upper:]' '[:lower:]')
# srt_controller -> SrtController, then SrtController_ActuatorsController
NEW_LIB="$(echo "$NEW_PKG" | sed -E 's/(^|_)([a-z])/\U\2/g')_${NEW_CLASS}"

echo "  package   : $OLD_PKG   ->  $NEW_PKG"
echo "  class     : $OLD_CLASS ->  $NEW_CLASS"
echo "  library   : $OLD_LIB   ->  $NEW_LIB"
echo "  namespace : $OLD_NS    ->  $NEW_NS"

# --- copy and rewrite -------------------------------------------------------

cp -r "$SRC" "$DEST"

[ -f "$DEST/src/${OLD_NS}.cpp" ]     && mv "$DEST/src/${OLD_NS}.cpp"     "$DEST/src/${NEW_NS}.cpp"
[ -f "$DEST/config/${OLD_NS}.yaml" ] && mv "$DEST/config/${OLD_NS}.yaml" "$DEST/config/${NEW_NS}.yaml"

while IFS= read -r f; do
  # `example_plugin_manager` is a DIFFERENT upstream MRS package cited in a
  # CMake comment. Without this guard the OLD_PKG substitution mangles it.
  sed -i 's/example_plugin_manager/@@KEEP@@/g' "$f"

  # longest first, so shorter names cannot corrupt the longer ones
  sed -i "s/${OLD_LIB}/${NEW_LIB}/g"     "$f"
  sed -i "s/${OLD_PKG}/${NEW_PKG}/g"     "$f"
  sed -i "s/${OLD_CLASS}/${NEW_CLASS}/g" "$f"
  sed -i "s/${OLD_NS}/${NEW_NS}/g"       "$f"

  sed -i 's/@@KEEP@@/example_plugin_manager/g' "$f"
done < <(grep -rlE "${OLD_LIB}|${OLD_PKG}|${OLD_CLASS}|${OLD_NS}" "$DEST" 2>/dev/null || true)

echo
echo "done -> $DEST"
echo "remaining references to the old names (expected: none):"
grep -rnE "${OLD_LIB}|\b${OLD_PKG}\b|${OLD_CLASS}|\b${OLD_NS}\b" "$DEST" 2>/dev/null | sed 's/^/  /' || echo "  none"
