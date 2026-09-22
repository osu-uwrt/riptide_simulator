#!/usr/bin/env python3
"""Build compact, metre-scale Polycase ML-22F competition light approximations.

Offline only: numpy/trimesh. +X is outward, origin is the clear lid center.
Dimensions follow Polycase's drawing; electronics/layout follow the supplied
competition photo and RoboNation's 16-LED build guide. See asset README.
"""
from pathlib import Path
import argparse
import numpy as np
import trimesh


def rounded_loop(width, height, radius, n=12):
    points=[]
    for cy,cz,start in ((width/2-radius,height/2-radius,0),
                        (-width/2+radius,height/2-radius,90),
                        (-width/2+radius,-height/2+radius,180),
                        (width/2-radius,-height/2+radius,270)):
        for a in np.radians(np.linspace(start,start+90,n,endpoint=False)):
            points.append([cy+radius*np.cos(a),cz+radius*np.sin(a)])
    return np.array(points)


def extrusion(loop, back, front, inner=None):
    n=len(loop);vertices=[];faces=[]
    for x in (back,front):vertices.extend([[x,y,z] for y,z in loop])
    for i in range(n):
        j=(i+1)%n
        faces.extend([[i,j,n+j],[i,n+j,n+i]])
    if inner is None:
        vertices.extend([[back,0,0],[front,0,0]])
        for i in range(n):
            j=(i+1)%n;faces.extend([[2*n,j,i],[2*n+1,n+i,n+j]])
    else:
        for x in (back,front):vertices.extend([[x,y,z] for y,z in inner])
        for i in range(n):
            j=(i+1)%n
            faces.extend([[2*n+i,3*n+j,2*n+j],[2*n+i,3*n+i,3*n+j],
                          [n+i,n+j,3*n+j],[n+i,3*n+j,3*n+i],
                          [i,2*n+j,j],[i,2*n+i,2*n+j]])
    mesh=trimesh.Trimesh(vertices,faces)
    mesh.fix_normals()
    return mesh


def build(output):
    output.mkdir(parents=True,exist_ok=True)
    groups={key:trimesh.Scene() for key in ('housing','cover','leds','robot_magnet')}
    gray=[.64,.66,.65,1];black=[.009,.012,.016,1];silver=[.62,.66,.70,1]
    purple=[.018,.004,.055,1];gold=[.55,.36,.08,1];white=[.85,.84,.74,1]

    def add(name,mesh,color,group='housing'):
        mesh.visual=trimesh.visual.texture.TextureVisuals(
            uv=np.zeros((len(mesh.vertices),2)),
            material=trimesh.visual.material.PBRMaterial(baseColorFactor=np.asarray(color,float),
                                                       roughnessFactor=.4))
        groups[group].add_geometry(mesh,node_name=name,geom_name=name)

    def box(name,center,size,color,group='housing',angle=0):
        mesh=trimesh.creation.box(size)
        mesh.apply_transform(trimesh.transformations.rotation_matrix(angle,[1,0,0]))
        mesh.apply_translation(center);add(name,mesh,color,group)

    def cylinder(name,start,end,radius,color,group='housing',sections=32):
        start=np.array(start);end=np.array(end);direction=end-start
        mesh=trimesh.creation.cylinder(radius,np.linalg.norm(direction),sections=sections)
        mesh.apply_transform(trimesh.geometry.align_vectors([0,0,1],direction))
        mesh.apply_translation((start+end)/2);add(name,mesh,color,group)

    outer=rounded_loop(.066675,.066548,.00589)
    inner=rounded_loop(.059,.059,.004)
    add('gray hollow base',extrusion(outer,-.0424688,-.014,inner),gray)
    add('back plate',extrusion(outer,-.0424688,-.040),gray)
    flange=rounded_loop(.0778,.051,.005)
    add('mounting flange',extrusion(flange,-.043,-.0405),gray)
    for sign in (-1,1):
        cylinder('flange hole '+str(sign),[-.0404,sign*.0358,0],[-.0401,sign*.0358,0],.0028,black)
    # Gasket and polished molded cover rim remain visible through the clear lid.
    add('silicone seal',extrusion(outer,-.0142,-.0134,inner),[.18,.19,.19,1])
    add('cover rim',extrusion(outer,-.0134,-.001,inner),[.65,.77,.83,.22],'cover')
    add('clear lid',extrusion(outer,-.001,0),[.78,.88,.94,.055],'cover')
    for y in (-.025,.025):
        for z in (-.025,.025):
            cylinder(f'screw boss {y} {z}',[-.014,y,z],[-.001,y,z],.0045,gray)
            cylinder(f'screw {y} {z}',[-.001,y,z],[.0008,y,z],.0027,silver)
            for angle in (0,np.pi/2):
                box(f'screw slot {y} {z} {angle}',[.0009,y,z],[.0002,.0037,.0005],black,angle=angle)
    circle=np.array([[.024*np.cos(a),.024*np.sin(a)] for a in np.linspace(0,2*np.pi,96,endpoint=False)])
    add('purple electronics board',extrusion(circle,-.011,-.009),purple)
    # Concentric PCB silk screen and discrete pads/parts are real geometry so
    # they retain perspective, occlusion and depth in both robot cameras.
    for radius in (.0155,.0228):
        a=np.linspace(0,2*np.pi,96,endpoint=False)
        loop=np.c_[np.cos(a),np.sin(a)]
        add('silkscreen '+str(radius),extrusion(loop*radius,-.0089,-.0088,loop*(radius-.00025)),white)
    box('magnetometer PCB',[-.0082,0,0],[.0013,.016,.014],purple)
    box('TLV493D sensor',[-.0074,0,0],[.001,.003,.003],black)
    for i in range(6):
        box('sensor pad '+str(i),[-.0074,-.006+i*.0024,-.005],[.0005,.001,.0016],gold)
    for i in range(16):
        a=2*np.pi*i/16;y,z=.0194*np.cos(a),.0194*np.sin(a)
        box('WS2812 package '+str(i),[-.0065,y,z],[.002,.005,.005],white,angle=a)
        box('LED lens '+str(i),[-.0053,y,z],[.0005,.0036,.0036],[1,1,1,1],'leds',a)
        for sign in (-1,1):
            yy,zz=y+sign*.0028*np.cos(a),z+sign*.0028*np.sin(a)
            box(f'LED contact {i} {sign}',[-.007,yy,zz],[.0005,.001,.0015],gold,angle=a)
        yy,zz=.013*np.cos(a),.013*np.sin(a)
        box('PCB trace '+str(i),[-.0087,yy,zz],[.0001,.0018,.0003],white,angle=a)
    for sign in (-1,1):
        cylinder('cable gland '+str(sign),[-.026,sign*.030,0],[-.026,sign*.045,0],.0065,black)
        cylinder('cable '+str(sign),[-.026,sign*.044,0],[-.026,sign*.066,-.010],.0025,black)
    # A short return loop and a strap suggest the shore-powered, pipe-mounted
    # node in the photo without introducing cables across the pool.
    a=np.linspace(-np.pi/2,np.pi/2,20)
    pts=np.c_[np.full_like(a,-.028),.066+.012*np.cos(a),.003+.013*np.sin(a)]
    for i in range(len(pts)-1):cylinder('return cable '+str(i),pts[i],pts[i+1],.0025,black,sections=12)
    box('pipe mounting strap',[-.047,0,0],[.004,.013,.05],black)
    # Tip origin follows the vehicle magnet TF plus robot_tip_offset. Stick/head dimensions
    # are visual estimates; they do not change the calibrated trigger radius.
    cylinder('printed stick',[0,0,.009],[0,0,.11],.006,[.045,.05,.06,1],'robot_magnet')
    cylinder('printed tip holder',[0,0,-.001],[0,0,.014],.019,[.08,.085,.09,1],'robot_magnet')
    cylinder('neodymium tip',[0,0,-.005],[0,0,.001],.016,silver,'robot_magnet')
    for key,scene in groups.items():
        scene.export(str(output/(key+'.glb')))
        print(key,sum(len(m.faces) for m in scene.geometry.values()),scene.bounds.tolist())


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    build(parser.parse_args().output)
