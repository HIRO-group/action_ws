import trimesh

mesh_objs = [
'link0.obj',
'link1.obj',
'link2.obj',
'link3.obj',
'link4.obj',
'link5.obj',
'link6.obj',
'link7.obj',
'hand.obj',
'finger.obj'
]

for idx, name in enumerate(mesh_objs):
  mesh = trimesh.load(name, process=False, force='mesh')
  convex = mesh.convex_hull
  convex.simplify_quadric_decimation(500)
  simple_name = name[:-4] + '_simple.obj'
  print(simple_name)
  convex.export(simple_name)
  print(mesh.is_convex)