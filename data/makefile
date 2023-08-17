
out/screenshots/%.mp4: out/screenshots/%
	ffmpeg -framerate 20 -pattern_type glob -i '$</*.png' \
	-vf "crop=1920:1042:0:0" \
	-c:v libx264 -crf 0 \
	$@
