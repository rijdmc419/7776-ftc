
#pragma version(1)
#pragma rs java_package_name(org.firstinspires.ftc.teamcode._Libs)
#pragma rs_fp_relaxed

// mapping kernel that posterizes an image in RGB565 format to a small set of saturated hues

// RenderScript kernel that performs RGB2HSV conversion
#if 0
uchar3 RS_KERNEL hsv(uchar3 in)
{
    uchar3 tempP;
    uchar minRGB = min( in.r, min( in.g, in.b ) );
    uchar maxRGB = max( in.r, max( in.g, in.b ) );
    uchar deltaRGB = maxRGB - minRGB;

    if ( deltaRGB <= 0) {

        tempP.s0 = 0; // undefined ???
        tempP.s1 = 0;

    } else { // deltaRGB > 0 -> maxRGB > 0

        tempP.s1 = (255 * deltaRGB) / maxRGB;

        if (in.r >= maxRGB) {

            if( in.g > in.b ) {

                tempP.s0 = (30 * (in.g - in.b)) / deltaRGB;        // between yellow & magenta

            } else {

                tempP.s0 = 180 + (30 * (in.g - in.b)) / deltaRGB;

            }
        } else if (in.g >= maxRGB) {

            tempP.s0 = 60 + (30 * (in.b - in.r)) / deltaRGB;  // between cyan & yellow

        } else {

            tempP.s0 = 120 + (30 * (in.r - in.g)) / deltaRGB;  // between magenta & cyan

        }

        /*
        if (in.b >= maxRGB) {
            if( in.g > in.r ) {
                tempP.s0 = (char)(30 * (in.g - in.r) / deltaRGB);        // between yellow & magenta
            } else {
                tempP.s0 = (char)(180 + 30 * (in.g - in.r) / deltaRGB);
            }
        } else if (in.g >= maxRGB) {
            tempP.s0 = (char)(60 + 30 * (in.r - in.b) / deltaRGB);  // between cyan & yellow
        } else {
            tempP.s0 = (char)(120 + 30 * (in.b - in.g) / deltaRGB);  // between magenta & cyan
        }
        */
    }

    tempP.s2 = maxRGB;

    return tempP;
}
#endif

int RS_KERNEL red(uchar3 pix) { return pix.r; }
int RS_KERNEL green(uchar3 pix) { return pix.g; }
int RS_KERNEL blue(uchar3 pix) { return pix.b; }

uchar3 RS_KERNEL toU3(int pix) {
    uchar3 u;
    u.r = (pix >> 16) & 0xFF;
    u.g = (pix >> 8) & 0xFF;
    u.b = pix & 0xFF;
    return u;
}

uchar3 RS_KERNEL domColor(uchar3 pix) {
            float n = 1.5F;    // dominance factor threshold
            uchar3 white = toU3(0xFFFFFF);
            uchar3 domClr = white;    // default is white (i.e. shades of gray)
/*          if (red(pix)>n*green(pix) && red(pix)>n*blue(pix))
                domClr = toU3(0xFF0000);     // red
            else
             if (blue(pix)>n*red(pix) && blue(pix)>n*green(pix))
                 domClr = toU3(0x0000FF);     // blue
            else
            if (green(pix)>n*red(pix) && green(pix)>n*blue(pix))
                domClr = toU3(0x00FF00);     // green
            else
            if (blue(pix)>n*red(pix) && green(pix)>n*red(pix))
                domClr = toU3(0x00FFFF);     // cyan
            else
            if (blue(pix)>n*green(pix) && red(pix)>n*green(pix))
                domClr = toU3(0xFF00FF);     // magenta
            else
*/
            if (red(pix)>n*blue(pix) && green(pix)>n*blue(pix) && red(pix)<n*green(pix) && green(pix)<n*red(pix))
                domClr = toU3(0xFFFF00);     // yellow

            // if it has no discernible hue, encode its gray level 0-7
            if (domClr.r == white.r && domClr.g == white.g && domClr.b == white.b) {
                float value = red(pix)*0.2f + green(pix)*0.7f + blue(pix)*0.1f; // 0..255
                domClr.r = domClr.g = domClr.b = (uchar)(value);  // return Value as shade of grey
            }
            return domClr;
}

/**
 * our regular 8 bit per channel RGB kernel
 */
uchar3 RS_KERNEL myKernelRgb(uchar3 in, uint32_t x, uint32_t y) {
    uchar3 out = domColor(in);
    return out;
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
