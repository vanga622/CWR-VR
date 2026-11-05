# CWR-VR: original model parts for the VR weapons, built from scratch.
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Run with Blender (4.x), no window:
#   blender -b --python resources/vr/make_parts.py
# Writes resources/vr/*.p3d (MLOD, SP3X levels, which the engine reads) and
# resources/vr/vr_metal.png.  Then, with PoseidonTools:
#   image convert vr_metal.png vr_metal.paa -f DXT1
#   image convert vr_parkerized.png vr_parkerized.paa -f DXT1
#   pbo pack <folder with the .p3d and .paa> resources/vr/vr.pbo
# The game mounts vr.pbo from next to its executable as prefix vr\.
#
# Everything here is made by this script: no game data goes in.
# Model axes as in the weapon models: +x towards the stock, +y up, +z right.
# Magazine: +z its top, +y across (as misc\mag_nato.p3d).

import math
import os
import random
import struct

import bmesh
import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
TEXTURE = "vr\\vr_metal.paa"
# Dark phosphate finish, as on the M16 receiver: two bands, carrier and cover.
TEXTURE_PARKERIZED = "vr\\vr_parkerized.paa"
UV_CARRIER = (1 / 4, 0.5)
UV_COVER = (3 / 4, 0.5)
# Texture atlas: four bands, steel, brass, copper, the dark inside of a receiver.
UV_STEEL = (1 / 8, 0.5)
UV_BRASS = (3 / 8, 0.5)
UV_COPPER = (5 / 8, 0.5)
UV_DARK = (7 / 8, 0.5)
UV_SPREAD = ((-0.1, -0.3), (0.1, -0.3), (0.1, 0.3), (-0.1, 0.3))
MAG_TOP_LENGTH = 0.07
MAG_TOP_WIDTH = 0.022
# The same two sizes are in SoldierOld.cpp, VRDrawWeaponPort.
PORT_LENGTH = 0.08
PORT_HEIGHT = 0.025
# The same two sizes are in SoldierOld.cpp, VRDrawWeaponPort.
BOLT_HEAD_LENGTH = 0.018
CARRIER_SEGMENT = 0.012
# How far the port stands off the receiver.  The game transforms in single
# precision: kilometres from the map's origin that is half a millimetre, and
# anything closer sinks into the receiver at some places on the map.
PORT_RISE = 0.0015


def new_object(name):
    mesh = bpy.data.meshes.new(name)
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    return obj


def box(bm, size, at=(0, 0, 0)):
    made = bmesh.ops.create_cube(bm, size=1.0)
    m = Matrix.Translation(Vector(at)) @ Matrix.Diagonal((*size, 1.0))
    bmesh.ops.transform(bm, matrix=m, verts=made["verts"])
    return made["verts"]


def cylinder(bm, radius, length, axis, at=(0, 0, 0), segments=12):
    made = bmesh.ops.create_cone(bm, cap_ends=True, segments=segments, radius1=radius, radius2=radius, depth=length)
    rot = {"x": Matrix.Rotation(math.pi / 2, 4, "Y"), "y": Matrix.Rotation(math.pi / 2, 4, "X"), "z": Matrix.Identity(4)}[axis]
    bmesh.ops.transform(bm, matrix=Matrix.Translation(Vector(at)) @ rot, verts=made["verts"])
    return made["verts"]


def bevel(bm, verts, width, segments=2):
    edges = list({e for v in verts for e in v.link_edges})
    bmesh.ops.bevel(bm, geom=list(verts) + edges, offset=width, segments=segments, affect="EDGES", profile=0.5)


def corner_normals(mesh, angle=math.radians(40)):
    """One normal per face corner: smooth across edges under the angle."""
    faces_of = {}
    for p in mesh.polygons:
        for v in p.vertices:
            faces_of.setdefault(v, []).append(p)
    cos = math.cos(angle)
    result = {}
    for p in mesh.polygons:
        for li, v in zip(p.loop_indices, p.vertices):
            n = Vector()
            for q in faces_of[v]:
                if p.normal.dot(q.normal) >= cos:
                    n += q.normal * q.area
            result[li] = n.normalized() if n.length > 0 else p.normal.copy()
    return result


def write_p3d(path, obj, uv_of_face, texture_name=TEXTURE):
    """MLOD with one level.  The engine's faces wind the other way round."""
    mesh = obj.data
    mesh.calc_loop_triangles()
    normals = corner_normals(mesh)
    points = [v.co.copy() for v in mesh.vertices]
    table = []
    index = {}

    def normal_index(n):
        key = (round(n.x, 4), round(n.y, 4), round(n.z, 4))
        if key not in index:
            index[key] = len(table)
            table.append(key)
        return index[key]

    faces = []
    for p in mesh.polygons:
        if p.loop_total > 4:
            raise ValueError("face with more than four corners in " + obj.name)
        uv = uv_of_face(p)
        corners = [(v, normal_index(normals[li])) for li, v in zip(p.loop_indices, p.vertices)]
        faces.append((list(reversed(corners)), uv))

    out = bytearray()
    out += b"MLOD" + struct.pack("<II", 0x0101, 1)
    out += b"SP3X" + struct.pack("<iiiiii", 28, 0x0100, len(points), len(table), len(faces), 0)
    for co in points:
        out += struct.pack("<fffi", co.x, co.y, co.z, 0)
    for n in table:
        out += struct.pack("<fff", -n[0], -n[1], -n[2])
    name = texture_name.encode("ascii").ljust(32, b"\0")
    for corners, uv in faces:
        out += name + struct.pack("<i", len(corners))
        for i in range(4):
            point, normal = corners[i] if i < len(corners) else (0, 0)
            # Spread inside the colour's band: the engine picks the texture's
            # detail from a face's area on it, and none leaves it untextured.
            du, dv = UV_SPREAD[i]
            out += struct.pack("<iiff", point, normal, uv[0] + du, uv[1] + dv)
        out += struct.pack("<i", 0)
    out += b"TAGG"
    # Parts are placed by their own origin.
    out += b"#Property#".ljust(64, b"\0") + struct.pack("<i", 128)
    out += b"autocenter".ljust(64, b"\0") + b"0".ljust(64, b"\0")
    out += b"#EndOfFile#".ljust(64, b"\0") + struct.pack("<i", 0)
    out += struct.pack("<f", 1.0)
    with open(path, "wb") as f:
        f.write(out)
    print("wrote", path, len(points), "points", len(faces), "faces")


def finish(obj, bm):
    bmesh.ops.triangulate(bm, faces=[f for f in bm.faces if len(f.verts) > 4])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(obj.data)
    bm.free()
    obj.data.update()


def steel(_face):
    return UV_STEEL


def handle(name, stem_length, stem_radius, knob):
    """A charging handle: stem out of the receiver's right side, then its knob."""
    obj = new_object(name)
    bm = bmesh.new()
    cylinder(bm, stem_radius, stem_length, "z", (0, 0, stem_length / 2))
    knob(bm, stem_length)
    finish(obj, bm)
    write_p3d(os.path.join(HERE, name + ".p3d"), obj, steel)


def svd_knob(bm, z):
    # Rounded, flattened grip.
    made = bmesh.ops.create_uvsphere(bm, u_segments=14, v_segments=10, radius=0.0085)
    m = Matrix.Translation(Vector((0, 0, z))) @ Matrix.Diagonal((1.0, 0.8, 0.75, 1.0))
    bmesh.ops.transform(bm, matrix=m, verts=made["verts"])


def vz58_knob(bm, z):
    # Short ribbed cylinder.
    cylinder(bm, 0.0065, 0.012, "z", (0, 0, z), 14)
    cylinder(bm, 0.0072, 0.002, "z", (0, 0, z + 0.003), 14)
    cylinder(bm, 0.0072, 0.002, "z", (0, 0, z - 0.003), 14)


def pistol_magazine(name):
    obj = new_object(name)
    bm = bmesh.new()
    # Body 28 mm front to back, 14 mm across, 125 mm long, leaning like a
    # double-stack box; floor plate at the bottom, follower lip at the top.
    body = box(bm, (0.028, 0.014, 0.118), (0, 0, 0))
    bevel(bm, [v for v in body if v.is_valid], 0.0018)
    plate = box(bm, (0.032, 0.0175, 0.005), (0.001, 0, -0.0615))
    bevel(bm, [v for v in plate if v.is_valid], 0.0012)
    lip = box(bm, (0.024, 0.0105, 0.004), (0, 0, 0.061))
    bevel(bm, [v for v in lip if v.is_valid], 0.0008)
    # The top round: brass case, bullet towards -x (the muzzle).
    cylinder(bm, 0.0048, 0.019, "x", (0.003, 0, 0.0665), 12)
    tip = bmesh.ops.create_cone(bm, cap_ends=True, segments=12, radius1=0.0047, radius2=0.0022, depth=0.007)
    bmesh.ops.transform(bm, matrix=Matrix.Translation(Vector((-0.0105, 0, 0.0665))) @ Matrix.Rotation(-math.pi / 2, 4, "Y"), verts=tip["verts"])
    finish(obj, bm)

    def uv(face):
        c = face.center
        return UV_BRASS if c.z > 0.0625 else UV_STEEL

    write_p3d(os.path.join(HERE, name + ".p3d"), obj, uv)


def magazine_top(name, rounds):
    """The top of a magazine, whose model is an open tube.

    Drawn below with x along and z across spanning -0.5..0.5, y out of the
    magazine in units of the width like z, then made MAG_TOP_LENGTH by
    MAG_TOP_WIDTH.  The game fits it to each magazine's opening (the same two
    sizes are in SoldierOld.cpp), so the rounds stay round however long the
    magazine is.  Bullets towards -x.
    """
    obj = new_object(name)
    bm = bmesh.new()
    # Follower: closes the opening, flush with its rim.
    box(bm, (1.0, 0.05, 1.0), (0, 0.005, 0))
    # Feed lips along the rear of the opening.
    for side in (-1, 1):
        box(bm, (0.62, 0.17, 0.09), (0.19, 0.085, side * 0.455))
    if rounds:
        # Two rounds, staggered like a double stack, the upper one on the right.
        for z, y, x in ((0.17, 0.19, 0.0), (-0.17, 0.07, 0.02)):
            cylinder(bm, 0.185, 0.56, "x", (x + 0.19, y, z), 14)
            neck = bmesh.ops.create_cone(bm, cap_ends=True, segments=14, radius1=0.185, radius2=0.115, depth=0.09)
            bmesh.ops.transform(bm, matrix=Matrix.Translation(Vector((x - 0.135, y, z))) @ Matrix.Rotation(-math.pi / 2, 4, "Y"), verts=neck["verts"])
            tip = bmesh.ops.create_cone(bm, cap_ends=True, segments=14, radius1=0.115, radius2=0.025, depth=0.27)
            bmesh.ops.transform(bm, matrix=Matrix.Translation(Vector((x - 0.315, y, z))) @ Matrix.Rotation(-math.pi / 2, 4, "Y"), verts=tip["verts"])
    bmesh.ops.transform(bm, matrix=Matrix.Diagonal((MAG_TOP_LENGTH, MAG_TOP_WIDTH, MAG_TOP_WIDTH, 1.0)), verts=bm.verts)
    finish(obj, bm)

    def uv(face):
        c = Vector((face.center.x / MAG_TOP_LENGTH, face.center.y / MAG_TOP_WIDTH, face.center.z / MAG_TOP_WIDTH))
        if c.y < 0.05 or abs(c.z) > 0.405:
            return UV_STEEL
        return UV_COPPER if c.x < -0.19 else UV_BRASS

    write_p3d(os.path.join(HERE, name + ".p3d"), obj, uv)


def port():
    """The open ejection port: a dark plate over the one painted on the receiver."""
    obj = new_object("vr_port")
    bm = bmesh.new()
    box(bm, (PORT_LENGTH, PORT_HEIGHT, PORT_RISE), (0, 0, PORT_RISE / 2))
    finish(obj, bm)
    write_p3d(os.path.join(HERE, "vr_port.p3d"), obj, lambda face: UV_DARK)


def carrier():
    """The bolt carrier seen in the port: rear end at the origin, towards -x."""
    obj = new_object("vr_carrier")
    bm = bmesh.new()
    box(bm, (PORT_LENGTH, PORT_HEIGHT * 0.86, 0.0012), (-PORT_LENGTH / 2, 0, PORT_RISE + 0.0012))
    # Two ribs along it.
    for y in (-0.2, 0.2):
        box(bm, (PORT_LENGTH, PORT_HEIGHT * 0.12, 0.0008), (-PORT_LENGTH / 2, PORT_HEIGHT * y, PORT_RISE + 0.0022))
    finish(obj, bm)
    write_p3d(os.path.join(HERE, "vr_carrier.p3d"), obj, steel)


def bolt_head():
    """The bolt ahead of the carrier, in the port of an M16: front at the origin, towards +x."""
    obj = new_object("vr_bolt_head")
    bm = bmesh.new()
    radius = 0.0078
    cylinder(bm, radius, BOLT_HEAD_LENGTH, "x", (BOLT_HEAD_LENGTH / 2, 0, PORT_RISE + 0.0018 - radius), 24)
    # The extractor along its side.
    box(bm, (BOLT_HEAD_LENGTH * 0.7, 0.0028, 0.0012), (BOLT_HEAD_LENGTH * 0.4, 0.0012, PORT_RISE + 0.002))
    finish(obj, bm)
    write_p3d(os.path.join(HERE, "vr_bolt_head.p3d"), obj, steel)


def carrier_segment(name, notches):
    """A length of the round bolt carrier: front at the origin, towards +x.

    Only its side shows above the port.  The game lays these one behind the
    other and cuts the last at the port's rear edge.
    """
    obj = new_object(name)
    bm = bmesh.new()
    radius = 0.024
    rise = PORT_RISE + 0.0024
    pitch = CARRIER_SEGMENT / max(notches, 1)
    for i in range(max(notches, 1)):
        ridge = pitch * 0.6 if notches else pitch
        cylinder(bm, radius, ridge, "x", (i * pitch + ridge / 2, 0, rise - radius), 48)
        if notches:
            # The forward assist's serrations.
            cut = pitch - ridge
            cylinder(bm, radius - 0.0009, cut, "x", (i * pitch + ridge + cut / 2, 0, rise - radius), 48)
    finish(obj, bm)
    write_p3d(os.path.join(HERE, name + ".p3d"), obj, lambda face: UV_CARRIER, TEXTURE_PARKERIZED)


def dust_cover():
    """The port's cover, shut: its hinge along the origin, the cover up from there."""
    obj = new_object("vr_dustcover")
    bm = bmesh.new()
    plate = box(bm, (PORT_LENGTH, PORT_HEIGHT, 0.001), (0, PORT_HEIGHT / 2, 0))
    bevel(bm, [v for v in plate if v.is_valid], 0.0003, 1)
    # The hinge pin, and the stamped rib that stiffens the cover.
    cylinder(bm, 0.0013, PORT_LENGTH, "x", (0, 0, 0), 10)
    box(bm, (PORT_LENGTH * 0.72, PORT_HEIGHT * 0.34, 0.0012), (0, PORT_HEIGHT * 0.55, 0.0009))
    finish(obj, bm)
    write_p3d(os.path.join(HERE, "vr_dustcover.p3d"), obj, lambda face: UV_COVER, TEXTURE_PARKERIZED)


def texture():
    """Blued steel, brass, copper and near black in bands, a little wear on all."""
    size = 64
    random.seed(1)
    image = bpy.data.images.new("vr_metal", size, size, alpha=False)
    pixels = []
    for _y in range(size):
        for x in range(size):
            grain = random.uniform(-0.012, 0.012)
            if x < size // 4:
                r, g, b = 0.20 + grain, 0.205 + grain, 0.215 + grain
            elif x < size // 2:
                r, g, b = 0.62 + grain * 3, 0.46 + grain * 3, 0.20 + grain * 2
            elif x < size * 3 // 4:
                r, g, b = 0.60 + grain * 3, 0.33 + grain * 3, 0.20 + grain * 2
            else:
                r, g, b = 0.012 + grain / 3, 0.012 + grain / 3, 0.014 + grain / 3
            pixels += [r, g, b, 1.0]
    image.pixels = pixels
    image.filepath_raw = os.path.join(HERE, "vr_metal.png")
    image.file_format = "PNG"
    image.save()


def parkerized():
    """The dark finish, carrier a little lighter than the cover."""
    size = 64
    random.seed(2)
    image = bpy.data.images.new("vr_parkerized", size, size, alpha=False)
    pixels = []
    for _y in range(size):
        for x in range(size):
            grain = random.uniform(-0.008, 0.008)
            if x < size // 2:
                r, g, b = 0.115 + grain, 0.12 + grain, 0.125 + grain
            else:
                r, g, b = 0.07 + grain, 0.09 + grain, 0.095 + grain
            pixels += [r, g, b, 1.0]
    image.pixels = pixels
    image.filepath_raw = os.path.join(HERE, "vr_parkerized.png")
    image.file_format = "PNG"
    image.save()


handle("vr_handle_svd", 0.024, 0.0032, svd_knob)
handle("vr_handle_vz58", 0.018, 0.003, vz58_knob)
pistol_magazine("vr_mag_pistol")
magazine_top("vr_mag_top", True)
magazine_top("vr_mag_top_empty", False)
port()
carrier()
bolt_head()
carrier_segment("vr_carrier_plain", 0)
carrier_segment("vr_carrier_notched", 3)
dust_cover()
texture()
parkerized()
