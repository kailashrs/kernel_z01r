make clean
export ARCH="arm64"
export PATH="/usr/lib/ccache/bin:$PATH"
export CROSS_COMPILE="ccache /home/kailashrs/5z/arm64-gcc/bin/aarch64-elf-"
export CROSS_COMPILE_ARM32="ccache /home/kailashrs/5z/arm32-gcc/bin/arm-eabi-"
make O=out Z01R_defconfig
make O=out -j$(nproc --all) 2>&1 | tee build.log
cd ..
cd AnyKernel3
rm zImage
rm dtbo.img
rm glitch3d.zip
python2 /home/kailashrs/5z/mkdtboimg.py create dtbo.img /home/kailashrs/5z/msm-4.9/out/arch/arm64/boot/dts/qcom/*.dtbo
cp /home/kailashrs/5z/msm-4.9/out/arch/arm64/boot/Image.gz-dtb zImage
zip -r glitch3d.zip *
