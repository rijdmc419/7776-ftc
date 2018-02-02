
#pragma version(1)
#pragma rs java_package_name(org.firstinspires.ftc.teamcode._Libs)
#pragma rs_fp_relaxed

// mapping kernel that posterizes an image in RGB565 format to a small set of saturated hues


/**
 * our regular 8 bit per channel RGB kernel
 */
uchar3 RS_KERNEL myKernelRgb(uchar3 in, uint32_t x, uint32_t y) {
    uchar3 ret = {in.b, in.r, in.g};
    // todo: do something more interesting than swapping channels
    return ret;
}

/**
 * rgb565 version of same, unfortunately the reflected layer ScriptC_rs565.forEach_myKernel565() will
 * only work with Element.U16, not with Element.RGB565, we use ScriptC_rs565_extension.java to get around those checks
 */
ushort RS_KERNEL myKernel565(ushort in565, uint32_t x, uint32_t y) {
    uchar3 inrgb = {
        (in565 >> 8) & 0xF8,
        (in565 >> 3) & 0xFC,
        (in565 << 3) & 0xF8
    };

    uchar3 outrgb = myKernelRgb(inrgb, x, y);

    ushort out565 =
          ((((uint16_t)outrgb.r) & 0xF8) << 8)
        | ((((uint16_t)outrgb.g) & 0xFC) << 3)
        | ((((uint16_t)outrgb.b) & 0xF8) >> 3)
    ;
    return out565;
}
