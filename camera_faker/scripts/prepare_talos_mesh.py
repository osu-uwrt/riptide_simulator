#!/usr/bin/env python3
"""Stream/simplify Talos CAD to a batched GLB, without modifying the source.

Offline dependencies: numpy, lxml, open3d, trimesh (not needed at runtime).
Usage: python3 prepare_talos_mesh.py Talos3.dae models/talos3/Talos3_body.glb --exclude-launcher
Preserves CAD coordinates and diffuse materials; CAD textures are not used.
"""
import argparse
import json
import re
from collections import defaultdict
from pathlib import Path
import numpy as np
from lxml import etree as E
import open3d as o3d
import trimesh


def weld_corners(corners, normals):
    # Group nearly identical normals without quantizing the exported shading.
    # This avoids storing three distinct vertices for every smooth triangle.
    packed = np.column_stack((corners.reshape(-1, 3), normals.reshape(-1, 3).round(2)))
    unique, inverse = np.unique(packed, axis=0, return_inverse=True)
    normals = normals.reshape(-1, 3)
    averaged = np.column_stack([
        np.bincount(inverse, weights=normals[:, axis], minlength=len(unique))
        for axis in range(3)])
    averaged /= np.maximum(np.linalg.norm(averaged, axis=1, keepdims=True), 1e-12)
    return unique[:, :3], inverse.reshape(-1, 3), averaged


def transfer_normals(vertices, faces, source_vertices, source_faces, source_normals):
    """Transfer CAD corner normals, keeping sharp seams and smooth curved faces.

    Probe slightly inside each output face so opposite sides of a sharp edge
    can select different source faces. Split vertices where normals differ;
    blindly averaging normals rounds off flat end caps and panel edges.
    """
    surface = o3d.t.geometry.RaycastingScene(nthreads=2)
    surface.add_triangles(
        o3d.core.Tensor(source_vertices.astype(np.float32)),
        o3d.core.Tensor(source_faces.astype(np.uint32)))
    corners = vertices[faces]
    probes = corners * .999 + corners.mean(axis=1, keepdims=True) * .001
    hit = surface.compute_closest_points(
        o3d.core.Tensor(probes.reshape(-1, 3).astype(np.float32)), nthreads=2)
    uv = hit['primitive_uvs'].numpy()
    weights = np.column_stack((1 - uv.sum(axis=1), uv))
    normals = np.einsum('ni,nij->nj', weights,
                        source_normals[hit['primitive_ids'].numpy()])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals /= np.maximum(lengths, 1e-12)
    return weld_corners(corners, normals)


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('source', type=Path)
parser.add_argument('output', type=Path)
parser.add_argument('--exclude-launcher', action='store_true',
                    help='Omit the torpedo/marker assembly and its loaded round for separate simulation rendering')
parser.add_argument('--triangle-ratio', type=float, default=.4,
                    help='Fraction of source triangles to retain per part (default: 0.4)')
parser.add_argument('--material-repairs', type=Path,
                    help='JSON rules reusing named CAD materials on placeholder-colored parts')
args = parser.parse_args()
if not 0 < args.triangle_ratio <= 1:
    parser.error('--triangle-ratio must be greater than zero and at most one')
source, output = args.source, args.output
effects, materials, geometries = {}, {}, {}
report = {'source': source.name, 'parts': [], 'source_triangles': 0}
report['triangle_ratio'] = args.triangle_ratio
report['normal_method'] = 'CAD corner normals interpolated onto simplified faces, preserving seams'
excluded = set()
report['excluded_parts'] = []
report['excluded_source_triangles'] = 0
report['material_repairs'] = []
repair_rules = json.loads(args.material_repairs.read_text()) if args.material_repairs else []
visual_scene = None
for _, e in E.iterparse(str(source), events=('end',), huge_tree=True):
    tag = E.QName(e).localname
    if tag == 'effect':
        value = e.find('.//{*}diffuse/{*}color')
        effects[e.get('id')] = tuple(np.fromstring(value.text, sep=' ')) if value is not None else (.4,.4,.4,1.)
        e.clear()
    elif tag == 'material':
        materials[e.get('id')] = e.find('{*}instance_effect').get('url')[1:]
        e.clear()
    elif tag == 'geometry':
        mesh = e.find('{*}mesh')
        # Match the same assembly as prepare_payload_mesh.py, including CAD's
        # flattened names. Exclude the entire mechanism, not just its covers.
        if args.exclude_launcher and 'torpedoes' in (e.get('name') or '').lower():
            excluded.add(e.get('id'))
            count = sum(int(t.get('count')) for t in mesh.findall('{*}triangles'))
            report['excluded_parts'].append({'id': e.get('id'), 'name': e.get('name')})
            report['excluded_source_triangles'] += count
            report['source_triangles'] += count
            e.clear()
            continue
        positions = {}
        for vertices in mesh.findall('{*}vertices'):
            node = vertices.find('{*}input[@semantic="POSITION"]')
            data = mesh.find('{*}source[@id="'+node.get('source')[1:]+'"]/{*}float_array')
            positions[vertices.get('id')] = np.fromstring(data.text, sep=' ').reshape(-1,3)
        parts=[]
        for triangles in mesh.findall('{*}triangles'):
            inputs = triangles.findall('{*}input')
            stride = max(int(i.get('offset')) for i in inputs)+1
            index = next(i for i in inputs if i.get('semantic')=='VERTEX')
            vertices = positions[index.get('source')[1:]]
            indices = np.fromstring(triangles.findtext('{*}p'),sep=' ',dtype=np.int32).reshape(-1,stride)
            faces = indices[:,int(index.get('offset'))].reshape(-1,3)
            normal_input = next((i for i in inputs if i.get('semantic') == 'NORMAL'), None)
            source_normals = None
            if normal_input is not None:
                data = mesh.find('{*}source[@id="'+normal_input.get('source')[1:]+'"]/{*}float_array')
                normals = np.fromstring(data.text, sep=' ').reshape(-1, 3)
                source_normals = normals[indices[:, int(normal_input.get('offset'))]].reshape(-1, 3, 3)
            report['source_triangles'] += len(faces)
            m=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices),o3d.utility.Vector3iVector(faces))
            m.remove_duplicated_vertices();m.remove_degenerate_triangles()
            target=max(160,int(len(faces)*args.triangle_ratio))
            if len(faces)>target:m=m.simplify_quadric_decimation(target, boundary_weight=10.)
            m.remove_unreferenced_vertices();m.compute_vertex_normals()
            result = (np.asarray(m.vertices).copy(), np.asarray(m.triangles).copy(),
                      np.asarray(m.vertex_normals).copy())
            if source_normals is not None:
                if len(faces) > target:
                    result = transfer_normals(result[0], result[1], vertices, faces, source_normals)
                else:
                    result = weld_corners(vertices[faces], source_normals)
            parts.append((*result, triangles.get('material')))
        geometries[e.get('id')]=(e.get('name'),parts)
        e.clear()
        if len(geometries)%100==0:print('Simplified',len(geometries),'parts',flush=True)
    elif tag=='visual_scene':visual_scene=e

if args.exclude_launcher and not excluded:
    raise ValueError('No torpedo/marker assembly found; refusing to export a body with a baked-in launcher')

batches=defaultdict(list)
for rule in repair_rules:
    if rule['material'] not in materials:
        raise ValueError('Unknown replacement CAD material: ' + rule['material'])

def visit(node,parent):
    matrix=parent.copy()
    for t in node:
        kind=E.QName(t).localname
        if kind=='matrix':matrix=matrix@np.fromstring(t.text,sep=' ').reshape(4,4)
        elif kind=='translate':matrix=matrix@trimesh.transformations.translation_matrix(np.fromstring(t.text,sep=' '))
        elif kind=='rotate':
            values=np.fromstring(t.text,sep=' ');matrix=matrix@trimesh.transformations.rotation_matrix(np.deg2rad(values[3]),values[:3])
        elif kind=='scale':matrix=matrix@np.diag([*np.fromstring(t.text,sep=' '),1])
    for instance in node.findall('{*}instance_geometry'):
        if instance.get('url')[1:] in excluded:
            continue
        name, parts=geometries[instance.get('url')[1:]]
        bindings={b.get('symbol'):b.get('target')[1:] for b in instance.findall('.//{*}instance_material')}
        bounds=[]
        for vertices,faces,normals,symbol in parts:
            material = bindings.get(symbol, symbol)
            for rule in repair_rules:
                if re.search(rule['geometry'], name, re.I) and re.search(rule['source_material'], material, re.I):
                    report['material_repairs'].append({'name': name, 'from': material, 'to': rule['material']})
                    material = rule['material']
                    break
            color=tuple(round(float(x),3) for x in effects[materials[material]])
            m=trimesh.Trimesh(vertices=vertices,faces=faces,vertex_normals=normals,process=False)
            m.apply_transform(matrix);batches[color].append(m);bounds.append(m.bounds)
        if bounds and any(k in (name or '').lower() for k in ('torpedo','marker','dropper','sinky')):
            b=np.asarray(bounds);report['parts'].append({'name':name,'min':b[:,0].min(axis=0).tolist(),'max':b[:,1].max(axis=0).tolist()})
    for child in node.findall('{*}node'):visit(child,matrix)
for node in visual_scene.findall('{*}node'):visit(node,np.eye(4))
scene=trimesh.Scene()
for i,(color,meshes) in enumerate(batches.items()):
    m=trimesh.util.concatenate(meshes)
    # Single-part batches take trimesh's copy path, which can drop cached
    # normals. Assign them explicitly so every material exports CAD shading.
    m.vertex_normals = np.vstack([part.vertex_normals for part in meshes])
    m.visual=trimesh.visual.texture.TextureVisuals(uv=np.zeros((len(m.vertices),2)),material=trimesh.visual.material.PBRMaterial(baseColorFactor=color,metallicFactor=.15,roughnessFactor=.5,alphaMode='BLEND' if color[3] < 1 else 'OPAQUE'))
    scene.add_geometry(m,node_name='CAD_material_%d'%i)
report['render_triangles']=sum(len(m.faces) for m in scene.geometry.values())
report['materials']=len(batches);report['bounds']=scene.bounds.tolist()
output.parent.mkdir(parents=True,exist_ok=True)
output.write_bytes(scene.export(file_type='glb'))
output.with_suffix('.json').write_text(json.dumps(report,indent=2))
print('RESULT',{k:v for k,v in report.items() if k not in ('parts', 'excluded_parts', 'material_repairs')},flush=True)
print('Repaired material assignments:', len(report['material_repairs']), flush=True)
