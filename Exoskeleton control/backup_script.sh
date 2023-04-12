cd /media/pi/data/

sudo dd bs=4M if=/dev/mmcblk0 | gzip > image1-`date +%d%m%y`.img.gz
