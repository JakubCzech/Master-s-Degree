from __future__ import annotations
file = '/home/jakub/Documents/GitHub/Master-s-Degree/vfo_tuning_fixed.text'
# for file in ['/home/jakub/Documents/GitHub/Master-s-Degree/AGV-Waypoint_Sender/test/NavfnPlanner.txt', '/home/jakub/Documents/GitHub/Master-s-Degree/AGV-Waypoint_Sender/test/SmacPlannerHybrid.txt', '/home/jakub/Documents/GitHub/Master-s-Degree/AGV-Waypoint_Sender/test/ThetaStarPlanner.txt']:
results_grid = {}
results_smac = {}
results_theta = {}
with open(file) as f:
    for line in f.readlines():
        if 'FAILED' in line:
            continue
        line = line[1:-2].replace(
            "' ",
            '',
        ).replace(" '", '').replace("'", '')
        line = line.split(',')
        for dict_, planner in zip([results_grid, results_smac, results_theta], ['GridBased', 'SmacPlanner', 'ThetaStarPlanner']):
            if planner not in line[0]:
                continue
            Kp = line[0].split(' ')[-1].split('_')[2]
            Lookahead = line[0].split(' ')[-1].split('_')[3]
            Rotate_to_heading = line[0].split(' ')[-1].split('_')[4]
            dict_[line[0]] = [*[int(i) for i in line[1:-1]], int(line[-1]), int(Kp), float(Lookahead), float(Rotate_to_heading)]
import pandas as pd

for dict_ in [results_grid, results_smac, results_theta]:
    df = pd.DataFrame.from_dict(
        dict_, orient='index', columns=[
            'P1', 'P2', 'P3', 'P4', 'P5', 'Total', 'Kp', 'Lookahead', 'Rotate_to_heading',
        ],
    )
    df['Total'] = df['Total'] / 10**9
    df['P1'] = df['P1'] / 10**9
    df['P2'] = df['P2'] / 10**9
    df['P3'] = df['P3'] / 10**9
    df['P4'] = df['P4'] / 10**9
    df['P5'] = df['P5'] / 10**9
    df.sort_values(by=['Total'], inplace=True)
    df['std'] = df.groupby(['Kp', 'Lookahead', 'Rotate_to_heading'])['Total'].transform('std')

    for row in df.iterrows():
        for i in range(0, 6):
            row[1][i] = round(row[1][i], 3)
        print(f'{row[0]} & {row[1][0]:.3f} & {row[1][1]:.3f} & {row[1][2]:.3f} & {row[1][3]:.3f} & {row[1][4]:.3f} & {row[1][5]:.3f} & {row[1][6]:.3f} &  {row[1][7] } & {row[1][8]}  & {row[1][9]} \\\\ \\hline')
        break
    print()
