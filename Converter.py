from PIL import Image

inpath = 'D:/FFOutput/320x240/Image{}.jpg'
outpath = 'D:/FFOutput/bin/Image{}.jpg'

for i in range(5478):
    im = Image.open(inpath.format(i + 1))
    f = open(outpath.format(i + 1), 'wb')
    for x in range(320):
        for y in range(240):
            R, G, B = im.getpixel((x, y))
            gray = ((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3)
            f.write(gray.to_bytes(2, 'big'))
    f.close()
