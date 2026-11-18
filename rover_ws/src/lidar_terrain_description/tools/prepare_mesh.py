#!/usr/bin/env python3
"""Offline binary-STL preparation; pip install numpy trimesh fast-simplification."""
import argparse, hashlib, json, struct
from pathlib import Path
import numpy as np
import trimesh
import fast_simplification

def main():
    p=argparse.ArgumentParser(); p.add_argument('input'); p.add_argument('--output',default='meshes'); args=p.parse_args()
    source=Path(args.input); out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
    with source.open('rb') as f:
        f.read(80); n=struct.unpack('<I',f.read(4))[0]
    if source.stat().st_size != 84+50*n: raise ValueError('Expected binary STL')
    a=np.memmap(source,mode='r',offset=84,dtype=[('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')],shape=(n,))
    v=a['v']; low=v.min(axis=(0,1)); high=v.max(axis=(0,1))
    near=(np.abs(v[:,:,0]).max(axis=1)<2)&(np.abs(v[:,:,1]).max(axis=1)<2)
    z0=float(np.median(v[near,:,2]))
    tile=(np.abs(v[:,:,0]).max(axis=1)<=30)&(np.abs(v[:,:,1]).max(axis=1)<=30)
    tv=v[tile].copy(); tv[:,:,2]-=z0
    mesh=trimesh.Trimesh(vertices=tv.reshape(-1,3),faces=np.arange(tv.size//3).reshape(-1,3),process=True)
    mesh.export(out/'terrain_tile.stl'); print('tile',len(mesh.faces),flush=True)
    print('welding overview',flush=True)
    vertices,index=np.unique(v.reshape(-1,3),axis=0,return_inverse=True)
    faces=index.reshape(-1,3); del index
    print('simplifying',len(vertices),len(faces),flush=True)
    sv,sf=fast_simplification.simplify(vertices,faces,target_count=200000)
    sv[:,2]-=z0
    simplified=trimesh.Trimesh(vertices=sv,faces=sf,process=False)
    simplified.export(out/'terrain_overview.stl')
    report={'source_filename':source.name,'source_sha256':hashlib.file_digest(source.open('rb'),'sha256').hexdigest(),
      'source_triangles':n,'source_bounds': [low.tolist(),high.tolist()], 'source_units':'unknown; default assumes metres',
      'z_offset_subtracted':z0,'tile_half_extent_source_units':30,'tile_triangles':len(mesh.faces),
      'overview_triangles':len(sf),'overview_method':'quadric edge collapse, target 200000 faces; visual approximation',
      'transform':'x=x_raw*scale; y=y_raw*scale; z=(z_raw-z_offset)*scale',
      'tile_method':'unchanged triangles with all vertices inside [-30,30] XY; no hole filling'}
    (out/'provenance.json').write_text(json.dumps(report,indent=2)+'\n'); print(report,flush=True)
if __name__=='__main__': main()
