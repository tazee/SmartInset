# Smart Inset Kit for Modo

A Modo kit for insetting selected polygons. It provides two modes: a self-intersection prevention algorithm and a parallel-offset subdivision algorithm.

## Command Version and Tool Version

The command version applies directly to selected polygons via `poly.smartInset`. The tool version adds **Safe Inset** to Polygon Mesh Operations and procedurally evaluates the inset for selected polygons. Edit the mode and parameters in the tool item's channels.

## Mode

- **Safe Inset:** A straight-skeleton-type inset based on internal angle bisectors and 3D collision detection. It automatically detects self-intersections and clamps the movement distance before an intersection occurs.
- **Padding Inset:** An inset based on parallel edge offsets and 2D intersection subdivision. It also supports concave polygons that split into multiple islands.

## Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `mode` | Selector | Select `safe` or `padding`. |
| `size` | Distance | Inset distance. |
| `threshold` | Numeric | (Safe Inset) Collision-limit detection margin. |
| `merge` | Boolean | (Safe Inset) Automatically merge nearby vertices. |
| `mergeDist` | Distance | (Safe Inset) Vertex merge-distance threshold. |
| `flat` | Boolean | (Padding Inset) Offset concave vertices flat. |
| `align` | Angle | (Padding Inset) Direction for inside/outside determination rays. |
| `connect` | Boolean | (Padding Inset) Connect the outer boundary edges with a side skirt. |
