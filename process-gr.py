
nyfile = open('USA-road-d.NY.gr', 'r')

outfile = open('ny-graph', 'w+')

for line in nyfile:
    linef = line.replace('a ', '')
    outfile.write(linef)
