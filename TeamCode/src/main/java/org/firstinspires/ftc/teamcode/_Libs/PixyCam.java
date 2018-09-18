package org.firstinspires.ftc.teamcode._Libs;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode._Libs.PixyCam.BlockType.CC_BLOCK;
import static org.firstinspires.ftc.teamcode._Libs.PixyCam.BlockType.NORMAL_BLOCK;
import static org.firstinspires.ftc.teamcode._Libs.PixyCam.BlockType.NO_START_CODE;

// device driver for PixyCam running normal streaming I2C protocol
// which should return multiple blocks of the same color if that's what's seen.
// see http://cmucam.org/projects/cmucam5/wiki/Porting_Guide
// TBD -- rewrite this code to actually do that ...

//@I2cSensor(name = "PixyCam", description = "PixyCam", xmlTag = "PixyCam")
@I2cDeviceType() @DeviceProperties(name = "PixyCam", description = "PixyCam", xmlTag = "PixyCam")
public class PixyCam extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    final int PIXY_ARRAYSIZE  =            100;
    final int PIXY_START_WORD    =         0xaa55;
    final int PIXY_START_WORD_CC   =       0xaa56;
    final int PIXY_START_WORDX    =        0x55aa;
    final int PIXY_SERVO_SYNC     =        0xff;
    final int PIXY_CAM_BRIGHTNESS_SYNC  =  0xfe;
    final int PIXY_LED_SYNC      =         0xfd;
    final int PIXY_OUTBUF_SIZE  =          64;
    final int PIXY_SYNC_BYTE     =         0x5a;
    final int PIXY_SYNC_BYTE_DATA   =      0x5b;

    // data types
    enum BlockType
    {
        NO_START_CODE,
        NORMAL_BLOCK,
        CC_BLOCK // color code block
    }


    /**
     * Block describes the signature, location, and size of a detected block.
     */
    public class Block
    {
        /**
         * A number from 1 through 7 corresponding to the color trained into the PixyCam,
         * or a sequence of octal digits corresponding to the multiple colors of a color code.
         */
        public final int signature;

        /**
         * The x, y location of the center of the detected block.
         * x is in the range (0, 255)
         *     0 is the left side of the field of view and 255 is the right.
         * y is in the range (0, 199)
         *     0 is the top of the field of view and 199 is the bottom.
         */
        public final int x, y;

        /**
         * The size of the detected block.
         * width or height of zero indicates no block detected.
         * maximum width is 255.
         * maximum height is 199.
         */
        public final int width, height;

        public Block(int signature, int x, int y, int width, int height)
        {
            this.signature = signature;
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
        }

        @Override public String toString()
        {
            return String.format("x: %d, y: %d, w: %d, h: %d", this.x, this.y, this.width, this.height);
        }

        public int checksum() {
            return signature+x+y+width+height;
        }
    }

    int g_skipStart = 0;
    BlockType g_blockType;
    ArrayList<Block> g_blocks;

    /**
     * The ReadWindow used to do a PixyCam I2C protocol GeneralQuery
     */
    private I2cDeviceSynch.ReadWindow legoProtocolGeneralQueryReadWindow;


    public PixyCam(I2cDeviceSynch deviceSynch)
    {
        super(deviceSynch, true);

        this.legoProtocolGeneralQueryReadWindow = new I2cDeviceSynch.ReadWindow(0x0, 1, I2cDeviceSynch.ReadMode.ONLY_ONCE);

        super.registerArmingStateCallback(false);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(1));
        this.deviceClient.engage();
    }


    private byte getByte() {
        return this.deviceClient.read8(0x50);
    }

    private int getWord()
    {
        // this routine assumes little endian
        byte c = getByte();
        int w = getByte();
        w <<= 8;
        w |= c;
        return w;
    }

    BlockType getStart()
    {
        int w, lastw;
        lastw = 0xffff;

        while(true)
        {
            w = getWord();
            if (w==0 && lastw==0) {
                return NO_START_CODE; // no start code
            }
            else if (w==PIXY_START_WORD && lastw==PIXY_START_WORD)
            {
                return NORMAL_BLOCK;
            }
            else if (w==PIXY_START_WORD_CC && lastw==PIXY_START_WORD)
            {
                return CC_BLOCK; // found color code block
            }
            else if (w==PIXY_START_WORDX)
            {
                getByte(); // we're out of sync! (backwards)
            }
            lastw = w;
        }
    }

    public int getBlocks(int maxBlocks)
    {
        ArrayList<Block> blocks;
        int w, blockCount, checksum, sum;

        if (g_skipStart == 0)
        {
            if (getStart()==NO_START_CODE)
                return -1;
        }
        else
            g_skipStart = 0;

        // make result array
        g_blocks = new ArrayList<Block>();

        for(blockCount=0; blockCount<maxBlocks; blockCount++)
        {
            checksum = getWord();
            if (checksum==PIXY_START_WORD) // we've reached the beginning of the next frame
            {
                g_skipStart = 1;
                g_blockType = NORMAL_BLOCK;
                return blockCount;
            }
            else if (checksum==PIXY_START_WORD_CC)
            {
                g_skipStart = 1;
                g_blockType = CC_BLOCK;
                return blockCount;
            }
            else if (checksum==0)
                return blockCount;

            Block b = new Block(getWord(), getWord(), getWord(), getWord(), getWord());
            g_blocks.add(b);

            // check checksum
            if (checksum==b.checksum())
                blockCount++;
            else
                return -2; //checksum error!

            w = getWord();
            if (w==PIXY_START_WORD)
                g_blockType = NORMAL_BLOCK;
            else if (w==PIXY_START_WORD_CC)
                g_blockType = CC_BLOCK;
            else
                return blockCount;
        }
        return blockCount;
    }

    public ArrayList<Block> getBlocks() {
        return g_blocks;
    }

    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "PixyCam";
    }
}