finding the src folder and keeping only that :

for dir in */; do
  [ -d "$dir" ] || continue
  tmpdir=$(mktemp -d)

  # Find all src folders inside this subdirectory
  find "$dir" -type d -name "src" | while read -r srcdir; do
    count=$(find "$tmpdir" -maxdepth 1 -type d | wc -l)
    dest="$tmpdir/src$((count))"
    mkdir -p "$dest"
    cp -r "$srcdir"/* "$dest"/ 2>/dev/null
  done

  # Delete everything inside the subfolder
  rm -rf "$dir"/*

  # Move all collected src folders back in
  mv "$tmpdir"/* "$dir"/ 2>/dev/null
  rmdir "$tmpdir" 2>/dev/null
done
