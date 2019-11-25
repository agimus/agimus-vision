import bpy, argparse, sys

parser = argparse.ArgumentParser(description='Generate COLLADA files from AprilTag.')
parser.add_argument('--cubesize', type=float, default=1., help='the size of the square')
parser.add_argument('input_tag', type=str, help='the input AprilTag')
parser.add_argument('output_dae', type=str, help='the output DAE file')

try:
    idx = sys.argv.index('--')
    remaining_args = sys.argv[idx+1:]
except:
    remaining_args = []
args = parser.parse_args(args=remaining_args)

cubesize = args.cubesize

bpy.ops.image.open(filepath=args.input_tag)
image = bpy.data.images[0]
assert image.size[0] == image.size[1]

def getPixel (image, i, j):
    """
    i and j be the horizontal and vertical coordinates.
    image.pixels is a list of floats of size 4 * image.size[0] * image.size[1].
    A column has image.size[0]. They are stored compactly in image.pixels (meaning
    that column 0 is image.pixels[0: 4 * image.size[0]])
    """
    assert j < image.size[0] and j >= 0
    assert i < image.size[1] and i >= 0
    index = (j * image.size[0] + i) * 4
    return image.pixels[index:index+4]
def isWhite (image, i, j):
    pixel = getPixel (image, i, j)
    return pixel[0] > 0.5

imagesize = image.size[0]
pixelsize = cubesize / imagesize

blackmat = bpy.data.materials.new(name="black")
blackmat.diffuse_color = (0,0,0)
blackmat.specular_color = (0,0,0)
whitemat = bpy.data.materials.new(name="white")
whitemat.diffuse_color = (1,1,1)
whitemat.specular_color = (1,1,1)

for i in range(imagesize):
    x = -cubesize/2 + (i+.5) * pixelsize
    for j in range(imagesize):
        y = -cubesize/2 + (j+.5) * pixelsize
        bpy.ops.mesh.primitive_plane_add (radius=pixelsize/2, location=(x,y,0))
        obj = bpy.context.object
        obj.name = 'pixel_{:0>2}_{:0>2}'.format(i,j)
        if isWhite(image, i, j):
            obj.data.materials.append(whitemat)
        else:
            obj.data.materials.append(blackmat)

bpy.ops.wm.collada_export(filepath=args.output_dae)
