from __future__ import annotations

import pandas as pd
file = '/home/jakub/Documents/GitHub/Master-s-Degree/vfo_2.text'
results = {}
with open(file) as f:
    for line in f.readlines():
        if 'FAILED' in line:
            continue
        else:
            line = line.split("'")
            controller = line[1].split(' ')[1].split('_')[1]
            planner = line[1].split(' ')[1].split('_')[0]
            results[line[1].replace(' ', '')] = [
                               *[int(line[i])
                for i in range(3, len(line)-1, 2)],
                controller,
                planner,
            ]
df = pd.DataFrame.from_dict(
    results, orient='index', columns=[
       'P1', 'P2', 'P3', 'P4', 'P5',  'Total','controller','planner'
    ],
)
df['Total'] = df['Total'] / 10**9
df['P1'] = df['P1'] / 10**9
df['P2'] = df['P2'] / 10**9
df['P3'] = df['P3'] / 10**9
df['P4'] = df['P4'] / 10**9
df['P5'] = df['P5'] / 10**9
df.sort_index(inplace=True)
for name in ['NEO', 'MPPI', 'RPP', 'VFO', 'DWB']:
    for row in df.iterrows():
        if name in row[0]:
            for i in range(0, 6):
                row[1][i] = round(row[1][i], 3)
            print(f'{row[0]} & {row[1][0]:.3f} & {row[1][1]:.3f} & {row[1][2]:.3f} & {row[1][3]:.3f} & {row[1][4]:.3f} & {row[1][5]:.3f} & {row[1][6]} & {row[1][7]}\\\\ \\hline')
    print()
print()

for name in [ 'Grid','Smac', 'Theta']:
    for row in df.iterrows():
        if name in row[0]:
            for i in range(0, 6):
                row[1][i] = round(row[1][i], 3)
            print(f'{row[0]} & {row[1][0]:.3f} & {row[1][1]:.3f} & {row[1][2]:.3f} & {row[1][3]:.3f} & {row[1][4]:.3f} & {row[1][5]:.3f}  \\\\ \\hline')
    print()

for name in ['NEO', 'MPPI', 'RPP', 'VFO', 'DWB']:
    for row in df.iterrows():
        if name in row[0]:
            for i in range(0, 6):
                row[1][i] = round(row[1][i], 3)
            print(f'{row[0]} &  {row[1][5]:.3f} \\\\ \\hline')
    print()
print()
