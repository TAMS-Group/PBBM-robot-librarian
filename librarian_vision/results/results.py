#!/usr/bin/env python3

#%%
import yaml
import numpy as np
from matplotlib import pyplot as plt

def get_matrix(path_to_data, yaml_file):
    with open(yaml_file, "r") as f:
        dirs = yaml.safe_load(f)

    scores = np.zeros((16, 15)) # 0 = invalid
    matches = np.zeros((16, 15)) # 0 = invalid

    for entry in dirs:
        dir = list(entry.keys())[0]
        book_ids = entry[dir]
        values = np.loadtxt(f"{path_to_data}/{dir}/values.txt")
        for id, row in zip(book_ids, values):
            scores[id] += row
            matches[id][np.argmax(row)] += 1

    for i in range(16):
        scores[i] = (scores[i] - np.min(scores[i])) 
        scores[i] /= np.max(scores[i])

    return scores, matches

enc_direct_scores, enc_direct_matches = get_matrix("/home/hpled/masters/librarian/encodings_data", "encodings_direct.yaml")
hsv_direct_scores, hsv_direct_matches = get_matrix("/home/hpled/masters/librarian/hsv_data", "hsv_direct.yaml")


#%%

# fix broken result format  

# paths = ['hsv/2023-01-20_18-07-48/1']
# ids = []

# with open("hsv_acc.yaml", "r") as file:
#     for line in file.readlines():
#         path, id = line.split(":")
#         paths.append(path)
#         ids.append(id)

# ids.append(" 0")

# with open("hsv_acc_fixed.yaml", "w") as file:
#     for p, id in zip(paths, ids):
#         file.write(p + ":" + id)

#%%

def get_matrix_acc(results_yaml):
    with open(results_yaml, "r") as file:
        res = yaml.safe_load(file)
    
    results = np.zeros((16, 15))
    
    for dir in res.keys():
        pred = int(dir.split("/")[-1])
        actual = res[dir]
        results[actual][pred - 1] += 1
    return results

acc_enc = get_matrix_acc("encodings_acc_fixed.yaml")
acc_hsv = get_matrix_acc("hsv_acc_fixed.yaml")

#%%

fig, ax = plt.subplots(nrows=1, ncols=4)

fig.set_size_inches(15, 5)
fig.set_dpi(200)

ax[0].imshow(hsv_direct_matches.T)
ax[0].set_title('HSV single')
ax[0].set_ylabel('Predicted books (ID)')

ax[1].imshow(enc_direct_matches.T)
ax[1].set_title("Encoding single")

ax[2].imshow(acc_hsv.T)
ax[2].set_title('HSV accumulated')

ax[3].imshow(acc_enc.T)
ax[3].set_title('Encoding accumulated')

for a in ax:
    a.set_yticks(range(0,15), range(1,16))
    a.set_xticks(range(0,16),['X', *[i for i in range(1,16)]], rotation=60)
    a.set_xlabel('True books (ID)')

fig.align_xlabels(ax)
plt.tight_layout()
plt.savefig("conf_matrix.png", bbox_inches='tight')

#%%
# compute multiclass accuracies
# using A = correct / all

def accuracies(m):
    m = m[1:]  # leave out missdetections
    return np.diagonal(m) / 150

a_hsv_direct = accuracies(hsv_direct_matches)
a_enc_direct = accuracies(enc_direct_matches)
a_hsv_acc = accuracies(acc_hsv)
a_enc_acc = accuracies(acc_enc)

print(f"hsv direct: {np.mean(a_hsv_direct):.4f} avg")
print(" & ".join([f"{i:.2f}" for i in a_hsv_direct]))

print(f"encoding direct: {np.mean(a_enc_direct):.4f} avg")
print(" & ".join([f"{i:.2f}" for i in a_enc_direct]))

print(f"hsv acc: {np.mean(a_hsv_acc):.4f} avg")
print(" & ".join([f"{i:.2f}" for i in a_hsv_acc]))

print(f"encoding acc: {np.mean(a_enc_acc):.4f} avg")
print(" & ".join([f"{i:.2f}" for i in a_enc_acc]))
