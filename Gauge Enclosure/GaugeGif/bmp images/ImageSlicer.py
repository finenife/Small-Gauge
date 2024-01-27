from PIL import Image, ImageChops

basepath = ("C:\\Users\\hayde\\Documents\\GitHub\\Small-Gauge\\Gauge Enclosure\\GaugeGif\\bmp images\\")

for i in range(1,35):
    try: 
        frame1=Image.open(f"{basepath}Frame {i}.bmp")
        frame2=Image.open(f"{basepath}Frame {i+1}.bmp")
    except:
        print("Some error in loading files, skipping")
        continue
    diff = ImageChops.difference(frame1,frame2)
    print(diff.getbbox())
    smallestImage = frame2.crop(diff.getbbox())

    smallestImage.save(f"{i}.bmp")
    
