# Python 2 script

FREECADPATH = '/usr/lib/freecad/lib'
import sys
sys.path.append(FREECADPATH)

ABS_THICKNESS = 2

try:
	import FreeCAD
except ValueError:
	print("Cannot open FreeCAD. Please, check the installation")
else:
	import Part
	from FreeCAD import Base
	from OpenSCAD2Dgeom import edgestofaces
	import importDXF
	import importDAE

	dxf_folder = "tigrillo_dxf/"
	stl_folder = "tigrillo_stl/"

	for name in ["femur"]:#, "femur", "tibia", "front", "hind", "middle"]:

		dxf_file = dxf_folder + name + ".dxf"
		stl_file = stl_folder + name + ".stl"
		doc_name = name

		doc = FreeCAD.newDocument(doc_name)
		importDXF.insert(dxf_file, doc_name)

		mesh_array = []
		for obj in doc.findObjects("Part::Feature"):
			shape = obj.Shape
			wire = Part.Wire(shape.Edges)
			face = Part.Face(wire)

		mesh_array.append(face.extrude(Base.Vector(0,0,3)))
		my_compound = Part.makeCompound(mesh_array)
		my_compound.exportStl(stl_file)