filenames = ['cube_red_rendered.csv', 'negative_train.csv',]
with open('training_data.csv', 'w') as outfile:
    for fname in filenames:
        with open(fname) as infile:
            for line in infile:
                outfile.write(line)
