package org.firstinspires.ftc.teamcode._Libs;

// image processing utility class that finds the biggest contiguous blob of pixels of a given color in an image

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Point;

public class BlobFinder {

    Bitmap image;
    boolean visited[][];
    int bbCount;            // pixel count of biggest connected blob found so far
    Point bbOrigin;         // origin of biggest connected blob found so far
    Point bMin, bMax;       // limits of connected blob at "current" pixel
    Point bbMin, bbMax;     // limits of biggest connected blob found so far
    int color;              // color we're looking for

    public BlobFinder(Bitmap bitmap) {
            image = bitmap;
            visited = new boolean[image.getHeight()][image.getWidth()];
            bbOrigin = new Point();
            bbMin = new Point();
            bbMax = new Point();
            bMin = new Point();
            bMax = new Point();
    }

    // return number of pixels in biggest blob of given color
    public int find(int c) {

        // save color we're looking for so we don't have to pass it recursively
        color = c;

        // Initialize the visited flag array
        for (int x=0; x<image.getWidth(); x++)
        for (int y=0; y<image.getHeight(); y++)
            visited[y][x] = false;

        // Initialize biggest blob count and min/max values
        bbCount = 0;
        final int infinity = 100000;
        bbMin.set(infinity, infinity);
        bbMax.set(-infinity, -infinity);

        // Search for blob starting at each Nth pixel
        final int sample=1;
        for (int x=0; x<image.getWidth(); x+=sample) {
            for (int y = 0; y < image.getHeight(); y+=sample) {
                // Deep travel that pixel
                bMin.set(infinity, infinity);
                bMax.set(-infinity, -infinity);
                int count = deepCount(x, y);

                // Check if it's the biggest so far ...
                if (count > bbCount) {
                    bbOrigin.x = x;
                    bbOrigin.y = y;
                    bbCount = count;
                    bbMin.set(bMin.x, bMin.y);
                    bbMax.set(bMax.x, bMax.y);
                }
            }
        }

        // Return the position and the size of the blob
        return bbCount;
    }

    // get x and y of blob centroid
    public Point getOrigin() { return bbOrigin; }
    public Point getCentroid() {
        return new Point((bbMin.x+bbMax.x)/2, (bbMin.y+bbMax.y)/2);
    }

    // This is the deep search function
    private int deepCount(int x, int y) {
        int count = 0;

        // Ignore the pixel if it's out of bounds
        if (x < 0 || x >= image.getWidth() || y < 0 || y >= image.getHeight())
            return count;

        // Ignore the pixel if it's already visited
        if (visited[y][x] == true)
            return count;

        // Mark the pixel as visited
        visited[y][x] = true;

        // Skip if the pixel is not the requested color
        if (image.getPixel(x,y) != color)
            return count;

        // Expand min/max to include this pixel
        if (x < bMin.x) bMin.x = x;
        if (x > bMax.x) bMax.x = x;
        if (y < bMin.y) bMin.y = y;
        if (y > bMax.y) bMax.y = y;

        // Increment count by 1
        count++;

        // Search for adjacent pixels (up, left, right, down)
        // recursively
        count += deepCount(x, y-1);
        count += deepCount(x-1, y);
        count += deepCount(x+1, y);
        count += deepCount(x, y+1);

        // return the total count
        return count;
    }
}
