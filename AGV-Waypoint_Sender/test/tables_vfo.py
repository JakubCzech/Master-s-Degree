from __future__ import annotations
file = '/home/jakub/Documents/GitHub/Master-s-Degree/AGV-Waypoint_Sender/test/NavfnPlanner.txt'
for file in ['/home/jakub/Documents/GitHub/Master-s-Degree/AGV-Waypoint_Sender/test/NavfnPlanner.txt', '/home/jakub/Documents/GitHub/Master-s-Degree/AGV-Waypoint_Sender/test/SmacPlannerHybrid.txt', '/home/jakub/Documents/GitHub/Master-s-Degree/AGV-Waypoint_Sender/test/ThetaStarPlanner.txt']:
    results = {}
    with open(file) as f:
        for line in f.readlines():
            if 'FAILED' in line:
                continue
            line = line[1:-2].replace(
                "' ",
                '',
            ).replace(" '", '').replace("'", '')
            line = line.split(',')
            results[line[0]] = [*[int(i) for i in line[1:-1]], int(line[-1])]
    import pandas as pd

    df = pd.DataFrame.from_dict(
        results, orient='index', columns=[
            'P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'Total',
        ],
    )
    df['Total'] = df['Total'] / 10**9
    df['P1'] = df['P1'] / 10**9
    df['P2'] = df['P2'] / 10**9
    df['P3'] = df['P3'] / 10**9
    df['P4'] = df['P4'] / 10**9
    df['P5'] = df['P5'] / 10**9
    df['P6'] = df['P6'] / 10**9
    df.sort_values(by=['Total'], inplace=True)

    for row in df.iterrows():
        for i in range(0, 7):
            row[1][i] = round(row[1][i], 3)
        print(f'{row[0]} & {row[1][0]:.3f} & {row[1][1]:.3f} & {row[1][2]:.3f} & {row[1][3]:.3f} & {row[1][4]:.3f} & {row[1][5]:.3f} & {row[1][6]:.3f} \\\\ \\hline')
        break
    print()
