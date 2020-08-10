export PATH="/usr/lib/ccache/bin:$PATH"

export KBUILD_BUILD_USER="spac3b0y"
export KBUILD_BUILD_HOST="TheHurtlocker"

export ARCH="arm64"

export CROSS_COMPILE="ccache aarch64-linux-gnu-"
export CROSS_COMPILE_ARM32="ccache arm-none-eabi-"

workdir="$HOME/5z"
srcdir="$HOME/5z/msm-4.9"
objdir="$srcdir/out/arch/arm64/boot"

build() {
 make clean
 make O=out Z01R_defconfig
 make O=out -j$(nproc --all) 2>&1 | tee build.log
}

name_zip() {
 kvstring="$(strings $objdir/Image | grep -m1 "Linux version")"

 kernelversion="$(cut -d' ' -f 3 <<< $kvstring)"
 glitchedversion="$(cut -d'-' -f 3 <<< $kernelversion)"

 buildcount="$(cut -d' ' -f 9 <<< $kvstring)"

 kernelname="$(cut -d'-' -f 2 <<< $kernelversion)"
 kernelname="${kernelname,,}"

 zipname="$kernelname-$glitchedversion-$buildcount"
}

package() {
 cd $workdir/AnyKernel3

 rm dtbo.img
 rm *.zip

 python2 $workdir/mkdtboimg.py create dtbo.img $objdir/dts/qcom/*.dtbo

 cp $objdir/Image.gz-dtb zImage

 zip -r $zipname.zip *
}

build
name_zip
package
