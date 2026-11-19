# python
import lx

print("=== MODO PYTHON TEST SCRIPT STARTED ===")

plugin_path = "/Users/taz/Desktop/dev/SmartInset/build/SmartInsetKit/extra/libsmartInset.dylib"
lx.eval(f'plugin.add filename:"{plugin_path}"')

kit_cfg = "/Users/taz/Desktop/dev/SmartInset/build/SmartInsetKit/index.cfg"
lx.eval(f'config.import filename:"{kit_cfg}"')

# Test 1: SmartInset on a flat square
lx.eval('scene.new')
lx.eval('tool.set prim.cube on')
lx.eval('tool.attr prim.cube minX -1.0')
lx.eval('tool.attr prim.cube maxX 1.0')
lx.eval('tool.attr prim.cube minY 0.0')
lx.eval('tool.attr prim.cube maxY 0.0')
lx.eval('tool.attr prim.cube minZ -1.0')
lx.eval('tool.attr prim.cube maxZ 1.0')
lx.eval('tool.apply')
lx.eval('tool.set prim.cube off')

lx.eval('select.type polygon')
lx.eval('select.all')

polys_before = lx.eval('query layerservice poly.N ? all')
print(f"Test 1 - Polygons before SmartInset: {polys_before}")

lx.eval('poly.smartInset mode:safe size:0.25')

polys_after = lx.eval('query layerservice poly.N ? all')
print(f"Test 1 - Total polygons after SmartInset: {polys_after}")

# Test 2: PaddingInset on a new scene
lx.eval('scene.new')
lx.eval('tool.set prim.cube on')
lx.eval('tool.attr prim.cube minX -1.0')
lx.eval('tool.attr prim.cube maxX 1.0')
lx.eval('tool.attr prim.cube minY 0.0')
lx.eval('tool.attr prim.cube maxY 0.0')
lx.eval('tool.attr prim.cube minZ -1.0')
lx.eval('tool.attr prim.cube maxZ 1.0')
lx.eval('tool.apply')
lx.eval('tool.set prim.cube off')

lx.eval('select.type polygon')
lx.eval('select.all')

polys_before_pad = lx.eval('query layerservice poly.N ? all')
print(f"Test 2 - Polygons before PaddingInset: {polys_before_pad}")

lx.eval('poly.smartInset mode:padding size:0.25 connect:true')

polys_after_pad = lx.eval('query layerservice poly.N ? all')
print(f"Test 2 - Total polygons after PaddingInset: {polys_after_pad}")

print("=== MODO PYTHON TEST SCRIPT COMPLETED SUCCESSFULLY ===")
lx.eval('app.quit')
