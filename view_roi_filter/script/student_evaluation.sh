#!/bin/bash
################################################################
if [ -z "$1" ]; then
    echo -e "please set input topic name "
    echo -e "you can type like: /carla/ego_vehicle/camera/front/image"
    exit 4
else
    INPUT_TOPIC="$1"
    OUTPUT_TOPIC="/filter/front/view_roi_filter$1"
fi

if [ -z "$2" ]; then
    echo -e "please set bagname without suffix"
    echo -e "you can type like: cross_left_sun"
    exit 4
else
    INPUT_BAG="$2.bag"
    OUTPUT_BAG="$2_roi.bag"
    OUTPUT_BLOCK_BAG="$2_roi_block.bag"
fi

if [ -z "$3" ]; then
    BITRATE=100000 # 100 KB/S
else
    BITRATE=$(($3 * 1000 / 5))
    echo -e "set constant bitrate with: $BITRATE"
fi

QP=25 # 25 32 37

# vmaf conf.
WIDTH=640
HEIGHT=480
PIXEL_FORMAT=420
BITDEPTH=8
MODEL_VERSION="vmaf_v0.6.1"
ADDITIONAL_FEATURE1="psnr"
ADDITIONAL_FEATURE2="float_ssim"

# extract input info
cd ../../../
echo -e "processing input images.............................................................................................."
roslaunch view_roi_filter export_image_from_bag.launch bag_name:="$INPUT_BAG" extract_topic:="$INPUT_TOPIC" >/dev/null 2>&1
cd src/view-adaptation/view_roi_filter/ || exit
mkdir -p data
cd data || exit
mkdir -p "$2"
cd "$2" || exit
mkdir -p input
mv ~/.ros/frame*.jpg input/
cd input || exit
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -framerate 10 -i frame%04d.jpg -y input_ref.yuv >/dev/null 2>&1
siti -of json --width $WIDTH --height $HEIGHT input_ref.yuv >siti.json

# set constant quantization parameters
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i input_ref.yuv -vcodec libx264 -bf 0 -qp $QP -y input_cqp.avi >/dev/null 2>&1
ffmpeg -i input_cqp.avi
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i input_ref.yuv -i input_cqp.avi -lavfi psnr="stats_file=psnr_cqp.log" -f null -
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i input_ref.yuv -i input_cqp.avi -lavfi ssim="stats_file=ssim_cqp.log" -f null -
ffmpeg -i input_cqp.avi -c:v rawvideo -pixel_format yuv420p -y input_dis_cqp.yuv >/dev/null 2>&1

vmaf --reference input_ref.yuv \
    --distorted input_dis_cqp.yuv \
    --width $WIDTH --height $HEIGHT --pixel_format $PIXEL_FORMAT --bitdepth $BITDEPTH \
    --model version=$MODEL_VERSION \
    --feature $ADDITIONAL_FEATURE1 \
    --feature $ADDITIONAL_FEATURE2 \
    --output vmaf_evaluation_cqp.xml

# set constant bitrates
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i input_ref.yuv -vcodec libx264 -bf 0 -b $BITRATE -y input_cbr.avi >/dev/null 2>&1
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i input_ref.yuv -i input_cbr.avi -lavfi psnr="stats_file=psnr_cbr.log" -f null -
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i input_ref.yuv -i input_cbr.avi -lavfi ssim="stats_file=ssim_cbr.log" -f null -
ffmpeg -i input_cbr.avi -c:v rawvideo -pixel_format yuv420p -y input_dis_cbr.yuv >/dev/null 2>&1

vmaf --reference input_ref.yuv \
    --distorted input_dis_cbr.yuv \
    --width $WIDTH --height $HEIGHT --pixel_format $PIXEL_FORMAT --bitdepth $BITDEPTH \
    --model version=$MODEL_VERSION \
    --feature $ADDITIONAL_FEATURE1 \
    --feature $ADDITIONAL_FEATURE2 \
    --output vmaf_evaluation_cbr.xml

# extract output info
cd ../../../../../../
echo -e "processing output images.............................................................................................."
roslaunch view_roi_filter export_image_from_bag.launch bag_name:="$OUTPUT_BAG" extract_topic:="$OUTPUT_TOPIC" >/dev/null 2>&1
cd src/view-adaptation/view_roi_filter/data/"$2" || exit
mkdir -p output
mv ~/.ros/frame*.jpg output/
cd output || exit
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -framerate 10 -i frame%04d.jpg -y output_ref.yuv >/dev/null 2>&1
siti -of json --width $WIDTH --height $HEIGHT output_ref.yuv >siti.json

# set constant quantization parameters
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i output_ref.yuv -vcodec libx264 -bf 0 -qp $QP -y output_cqp.avi >/dev/null 2>&1
ffmpeg -i output_cqp.avi
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i output_ref.yuv -i output_cqp.avi -lavfi psnr="stats_file=psnr_cqp.log" -f null -
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i output_ref.yuv -i output_cqp.avi -lavfi ssim="stats_file=ssim_cqp.log" -f null -
ffmpeg -i output_cqp.avi -c:v rawvideo -pixel_format yuv420p -y output_dis_cqp.yuv >/dev/null 2>&1

vmaf --reference output_ref.yuv \
    --distorted output_dis_cqp.yuv \
    --width $WIDTH --height $HEIGHT --pixel_format $PIXEL_FORMAT --bitdepth $BITDEPTH \
    --model version=$MODEL_VERSION \
    --feature $ADDITIONAL_FEATURE1 \
    --feature $ADDITIONAL_FEATURE2 \
    --output vmaf_evaluation_cqp.xml

# set constant bitrates
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i output_ref.yuv -vcodec libx264 -bf 0 -b $BITRATE -y output_cbr.avi >/dev/null 2>&1
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i output_ref.yuv -i output_cbr.avi -lavfi psnr="stats_file=psnr_cbr.log" -f null -
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i output_ref.yuv -i output_cbr.avi -lavfi ssim="stats_file=ssim_cbr.log" -f null -
ffmpeg -i output_cbr.avi -c:v rawvideo -pixel_format yuv420p -y output_dis_cbr.yuv >/dev/null 2>&1

vmaf --reference output_ref.yuv \
    --distorted output_dis_cbr.yuv \
    --width $WIDTH --height $HEIGHT --pixel_format $PIXEL_FORMAT --bitdepth $BITDEPTH \
    --model version=$MODEL_VERSION \
    --feature $ADDITIONAL_FEATURE1 \
    --feature $ADDITIONAL_FEATURE2 \
    --output vmaf_evaluation_cbr.xml

# extract block-based output info
cd ../../../../../../
echo -e "processing block-based output images.............................................................................................."
roslaunch view_roi_filter export_image_from_bag.launch bag_name:="$OUTPUT_BLOCK_BAG" extract_topic:="$OUTPUT_TOPIC" >/dev/null 2>&1
cd src/view-adaptation/view_roi_filter/data/"$2" || exit
mkdir -p output_block
mv ~/.ros/frame*.jpg output_block/
cd output_block || exit
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -framerate 10 -i frame%04d.jpg -y output_ref.yuv >/dev/null 2>&1
siti -of json --width $WIDTH --height $HEIGHT output_ref.yuv >siti.json

# set constant quantization parameters
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i output_ref.yuv -vcodec libx264 -bf 0 -qp $QP -y output_cqp.avi >/dev/null 2>&1
ffmpeg -i output_cqp.avi
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i output_ref.yuv -i output_cqp.avi -lavfi psnr="stats_file=psnr_cqp.log" -f null -
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i output_ref.yuv -i output_cqp.avi -lavfi ssim="stats_file=ssim_cqp.log" -f null -
ffmpeg -i output_cqp.avi -c:v rawvideo -pixel_format yuv420p -y output_dis_cqp.yuv >/dev/null 2>&1

vmaf --reference output_ref.yuv \
    --distorted output_dis_cqp.yuv \
    --width $WIDTH --height $HEIGHT --pixel_format $PIXEL_FORMAT --bitdepth $BITDEPTH \
    --model version=$MODEL_VERSION \
    --feature $ADDITIONAL_FEATURE1 \
    --feature $ADDITIONAL_FEATURE2 \
    --output vmaf_evaluation_cqp.xml

# set constant bitrates
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i output_ref.yuv -vcodec libx264 -bf 0 -b $BITRATE -y output_cbr.avi >/dev/null 2>&1
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i output_ref.yuv -i output_cbr.avi -lavfi psnr="stats_file=psnr_cbr.log" -f null -
ffmpeg -pix_fmt yuv420p -s ${WIDTH}x${HEIGHT} -i output_ref.yuv -i output_cbr.avi -lavfi ssim="stats_file=ssim_cbr.log" -f null -
ffmpeg -i output_cbr.avi -c:v rawvideo -pixel_format yuv420p -y output_dis_cbr.yuv >/dev/null 2>&1

vmaf --reference output_ref.yuv \
    --distorted output_dis_cbr.yuv \
    --width $WIDTH --height $HEIGHT --pixel_format $PIXEL_FORMAT --bitdepth $BITDEPTH \
    --model version=$MODEL_VERSION \
    --feature $ADDITIONAL_FEATURE1 \
    --feature $ADDITIONAL_FEATURE2 \
    --output vmaf_evaluation_cbr.xml
