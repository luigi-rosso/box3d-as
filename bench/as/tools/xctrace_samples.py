#!/usr/bin/env python3
# Flattens an xctrace time-profile export to one line per sample: weight (us), then PCs leaf first.
#   xctrace_samples.py <time-profile.xml> > samples.txt
import sys
import xml.etree.ElementTree as ET

frames = {}
backtraces = {}
weights = {}
for _, el in ET.iterparse(sys.argv[1], events=('end',)):
    if el.tag != 'row':
        continue
    weight = 0
    pcs = []
    for child in el:
        if child.tag == 'weight':
            if 'ref' in child.attrib:
                weight = weights[child.attrib['ref']]
            else:
                weight = int(child.text) // 1000
                weights[child.attrib['id']] = weight
        elif child.tag == 'tagged-backtrace':
            bt = child.find('backtrace')
            if bt is None:
                continue
            if 'ref' in bt.attrib:
                pcs = backtraces[bt.attrib['ref']]
            else:
                for f in bt.findall('frame'):
                    if 'ref' in f.attrib:
                        pcs.append(frames[f.attrib['ref']])
                    else:
                        addr = int(f.attrib['addr'], 16) - 1
                        frames[f.attrib['id']] = addr
                        pcs.append(addr)
                backtraces[bt.attrib['id']] = pcs
    if pcs:
        print(weight, ' '.join(f'{pc:x}' for pc in pcs))
    el.clear()
