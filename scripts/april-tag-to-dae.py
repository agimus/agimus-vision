#!/usr/bin/python3

import argparse, imageio, datetime, os

parser = argparse.ArgumentParser(description='Generate COLLADA files from AprilTag.')
parser.add_argument('--cubesize', type=float, default=1., help='the size of the square in meters (excluding the white border)')
parser.add_argument('--input-tag', type=str, default=None, help='URL of the input AprilTag')
parser.add_argument('--family', type=str, default="36h11", help='The AprilTag family')
parser.add_argument('--number', type=int, default=None, help='The AprilTag number')
parser.add_argument('--output-dae', type=str, default=None, help='the output DAE file (computed if not provided)')
parser.add_argument('--template-dae', type=str, default=os.path.dirname(__file__)+'/april-tag-template.dae', help='the template DAE file')

url_fmt="""https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag{family}/tag{family_}_{number:0>5}.png"""
node_fmt = """
      <node id="pixel_{name}" name="pixel_{name}" type="NODE">
        <matrix sid="transform">1 0 0 {x} 0 1 0 {y} 0 0 1 0 0 0 0 1</matrix>
        <instance_geometry url="#plane_{name}" name="{name}">
          <bind_material>
            <technique_common>
              <instance_material symbol="{color}-material" target="#{color}-material"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>"""
geometry_fmt = """
    <geometry id="plane_{name}" name="plane_{name}">
      <mesh>
        <source id="{name}-positions">
          <float_array id="{name}-positions-array" count="12">-{pixelhalfside} -{pixelhalfside} 0 {pixelhalfside} -{pixelhalfside} 0 -{pixelhalfside} {pixelhalfside} 0 {pixelhalfside} {pixelhalfside} 0</float_array>
          <technique_common>
            <accessor source="#{name}-positions-array" count="4" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <source id="{name}-normals">
          <float_array id="{name}-normals-array" count="3">0 0 1</float_array>
          <technique_common>
            <accessor source="#{name}-normals-array" count="1" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>
        <vertices id="{name}-vertices">
          <input semantic="POSITION" source="#{name}-positions"/>
        </vertices>
        <polylist material="{color}-material" count="2">
          <input semantic="VERTEX" source="#{name}-vertices" offset="0"/>
          <input semantic="NORMAL" source="#{name}-normals" offset="1"/>
          <vcount>3 3 </vcount>
          <p>1 0 2 0 0 0 1 0 3 0 2 0</p>
        </polylist>
      </mesh>
    </geometry>"""

args = parser.parse_args()
if args.input_tag is not None:
    input_img = args.input_tag
    if args.output_dae is None:
        k = input_img.rfind('.')
        output_dae = input_img[:k] + '.dae'
else:
    input_img = url_fmt.format(
            family=args.family,
            family_=args.family.replace('h', '_'),
            number=args.number)
    if args.output_dae is None:
        output_dae = "tag{family_}_{number:0>5}.dae".format(
            family_=args.family.replace('h', '_'),
            number=args.number)
    print('Accessing', input_img)
if args.output_dae is None:
    print("Writing to", output_dae)

cubesize = args.cubesize

image = imageio.imread(input_img)
assert image.shape[0] == image.shape[1]

def isWhite (image, i, j):
    pixel = image[i,j]
    return pixel[0] > 127

imagesize = image.shape[0]
cubesize_wb = cubesize * imagesize / (imagesize - 2)
pixelsize = cubesize_wb / imagesize

geometries = ""
nodes = ""
for i in range(imagesize):
    x = -cubesize_wb/2 + (i+.5) * pixelsize
    for j in range(imagesize):
        y = -cubesize_wb/2 + (j+.5) * pixelsize
        color='white' if isWhite(image,i,j) else 'black'
        geometries += geometry_fmt.format(
                name = '{:0>2}_{:0>2}'.format(i,j),
                pixelhalfside = pixelsize/2,
                color=color)
        nodes += node_fmt.format(
                name = '{:0>2}_{:0>2}'.format(i,j),
                x = y, y = -x,
                color=color)

with open(output_dae, 'w') as fout:
    with open(args.template_dae, 'r') as ftpl:
        for tpl in ftpl.readlines():
            fout.write(tpl.format(date=datetime.datetime.now(),
                nodes=nodes, geometries=geometries, pixelhalfside=pixelsize/2))
