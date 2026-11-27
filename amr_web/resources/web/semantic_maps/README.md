# Semantic Warehouse Scene v1

Use the exact map ID returned by `GET /api/maps`, which is the saved map YAML's
filename stem. If Navigation uses `<map_id>.yaml`, save the scene as
`~/project_ws/maps/<map_id>/semantic_scene.json` next to its route storage.
The 3D view requests `GET /api/map/scene/{map_id}`. If the local file is absent,
the server checks the packaged `semantic_maps/<map_id>.json` directory for
compatibility with existing deployments. If both are absent, the selected map
renders using automatic geometry. The local file can be added or updated on the
server without rebuilding. Select the map again or reload the page to fetch it.

The map's origin, resolution and yaw describe the occupancy raster. Object poses
are in its ROS map frame, in metres and radians about +Z. Z is at the bottom of
each object. The layout's `map_id` and `frame_id` must match the selected map.
Legacy files without `schema_version` or `frame_id` are accepted.

```json
{
  "schema_version": 1,
  "map_id": "replace_with_navigation_map_id",
  "frame_id": "map",
  "objects": [
    {
      "id": "rack_1", "type": "rack",
      "pose": {"x": 2.0, "y": 1.0, "yaw": 0.4},
      "width": 2.0, "depth": 0.6, "height": 1.8,
      "slots": [
        {"id": "slot_1", "relative_to": "rack", "pose": {"x": -0.5, "y": 0, "z": 0.6}}
      ]
    },
    {
      "id": "pallet_1", "type": "pallet",
      "parent": {"type": "rack_slot", "rack_id": "rack_1", "slot_id": "slot_1"},
      "boxes": [
        {"id": "box_1", "width": 0.3, "depth": 0.2, "height": 0.4,
         "pose": {"x": 0, "y": 0, "z": 0}}
      ]
    }
  ]
}
```

Top-level object IDs are unique per map. Slot IDs are scoped to a rack. Box
IDs under `boxes` are scoped to their pallet or rack slot. Legacy child boxes
without an ID use their position in the list as a fallback ID. Assign explicit
IDs before using a future editor. Rack and slot poses also accept the legacy flat
`x/y/z/yaw` fields. Slots default to map-frame poses. Use
`"relative_to": "rack"` for slots that should move and rotate with the rack.
Slot box offsets are relative to the slot pose. Pallet box offsets are relative
to the pallet top. Display clamps negative pallet-box Z offsets to the deck. A top-level `box` with a map-frame pose represents a freestanding box.

Interior occupancy appears on the floor texture and in the low LiDAR-height strip when the existing TF is available. The strip excludes rack, pallet and cargo overlaps. The 3D wall layer comes only
from observed occupied perimeter bands. If a perimeter intersects a rack, the
wall mesh is clipped to the rack footprint and the console identifies the rack for coordinate verification. The original OccupancyGrid is shared unchanged with
the rest of the system. The viewer accepts legacy `overrides` in scene files
for compatibility. They do not add or remove perimeter evidence. Confirmed
rack geometry still suppresses overlapping cloud estimates.

## Pallet state API

A pallet with a configured parent is placed on initial load. A pallet with
no parent is a template for runtime state. The pallet visualization uses the
surveyed 0.40 × 0.30 × 0.05 m envelope across all states. The existing `/ws`
accepts the following shape as a **server-to-client** snapshot from a future
producer with verified cargo identity and pose.

```json
{
  "type": "semantic_state", "map_id": "replace_with_navigation_map_id", "frame_id": "map",
  "objects": [
    {"id": "pallet_1", "type": "pallet",
     "parent": {"type": "robot"}, "pose": {"x": 0, "y": 0, "z": 0}}
  ]
}
```

A `rack_slot` parent accepts an optional slot-local pose. A `robot` parent
accepts an optional pose relative to the fork attachment. Its zero offset
seats the procedural pallet deck on the GLB tine surfaces. A `map` parent
requires a map-frame pose for a destination. The corresponding states are
`rack`, `carried` and `destination`. Legacy snapshots can supply these states
and the corresponding rack and slot IDs in place of `parent`. Supply
child boxes and their dimensions inline or inherit them from the same pallet ID in
the config. A runtime snapshot becomes authoritative for rendered pallets.
Runtime pallet geometry clears after 3 seconds without a valid update or on
WebSocket disconnect. A configured pose from before runtime ownership does not
reappear while that scene is mounted. No mission adapter is required to load a layout.

## Future Edit Layout

Keep the editable draft in the versioned JSON schema. Pick objects by ID,
edit map-frame geometry or parent-local offsets, validate with
`normalizeSemanticLayout` and preview through the same viewer. A future save
API can write the same map-scoped JSON atomically and use a revision or ETag
to reject concurrent edits. This version provides layout reads and runtime
state consumption.

## Live shelf and cargo

For the active managed map and route, the read endpoint also derives
`shelf_front` from the installed mission `config/shelf.yaml` using the existing
`ShelfModel` and route anchor. The browser passes the selected route ID.
The model uses the surveyed 1.875 m width, 0.90 m overall height, 0.35 m slot
depth and the 0.20/0.55 m support heights. Vertical partitions follow the
configured columns. Slot centres are halfway into the opening in the approach
direction. A Retrieval selection marks the existing pallet identity at its
slot support. The camera face pose is retained in its metadata. The pallet extends 0.30 m into the 0.35 m shelf and spans 0.40 m across the opening.
Plate and partition thicknesses are visual conventions.

Automatic geometry uses the shared default mission config deployed by the
current launch. A route has to contain the configured shelf anchor in the map
frame. Review any custom ROS `shelf_config` override before using this scene.
A confirmed mission `shelf_slot_id` links a rendered slot to the authoritative
shelf status. Fresh `full` shows one pallet and cargo, `empty` hides them, and
unconfirmed or stale data marks the support surface unknown.

Optional surveyed cargo dimensions belong in the map's scene catalog:

```json
{"cargo_catalog": [
  {"cargo_type": "example_type", "width": 0.30, "depth": 0.25,
   "height": 0.40, "model": "box", "color": "#a29483"}
]}
```

The example is a schema illustration. Use verified dimensions for real entries.
Catalog sizes default to `dimensions_source: "configured"`. Use `"measured"`
for surveyed dimensions. Width and depth are carton-local X and Y in metres.
An optional `yaw` rotates the carton on the pallet in radians. For live slots,
`cargo_height_m` sets the nominal display height. The default Shelf Config
adapter supplies 0.15 m for A slots and 0.40 m for B slots. The original
camera height remains in metadata and is used when the pallet is represented
on the floor or carried. With an unknown footprint, the opaque carton uses
a bounded placeholder. Shelf placeholders contribute no extra cloud
suppression volume. Detector-selected floor loads use the rendered volume.

The viewer checks occupied slot cartons against the rendered shelf plates,
partitions and posts, using the pallet bottom at the support and the carton
bottom on the deck. Actual configured dimensions that exceed that clearance
produce `warehouse_fit_issues` in the map summary and a console warning. The
pallet can remain on the support while its conflicting carton is omitted.
The comparison uses the rendered construction. Physical clearances require
surveyed slot dimensions.

Confirmed loads use an oriented pallet-and-carton envelope to find overlap
candidates. Suppression accounts for occupied parts inside that envelope,
and trims partial overlaps using the part volumes.
Cartons use kraft faces, a folded lid and two tape bands. An optional catalog
color tints the tape. For observed pallet poses, the detector reports the face
centre. The viewer places the origin at the centre of the pallet's bottom plane,
0.15 m inward along detector yaw and 0.025 m below the observed face centre.
Local +X follows the approach and fork direction. The orange pallet has
five open deck boards and three longitudinal runners.

`system_status.warehouse_view` carries the existing mission and shelf evidence
with `map_id` and `frame_id`. A selected Retrieval target uses its source slot
identity where available. A floor target uses an operation-scoped temporary
ID. Its preview starts from the detector-confirmed map pose and can use a recent /pallet/cargo estimate until the finalized mission height is available. The viewer blends small pose changes during Detect and holds the pose at the mission lock. The same pallet and carton are moved to the fork attachment on confirmed pickup. Sensor overlap during floor PICKUP uses the displayed pallet/carton volumes without expanding the estimated carton footprint. The bridge tracks confirmed carriage and rack delivery for display only.
After confirmed retrieval release at D_2, the client displays the pallet on the floor for three seconds from confirmation. When the release lacks a measured pose it uses the last valid map transform of the carried load and marks the placement estimated. For rack DROP, completed release transfers ownership to the destination slot, even while the occupancy update is pending. A departed source remains hidden while its sensor still
reports the pre-pickup full reading. New WebSocket sessions receive the same bridge
snapshot, and the projection clears on map change. Pallet IDs are display
identities, with no persisted inventory or serial-number database.
