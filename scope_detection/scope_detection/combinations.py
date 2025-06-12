import numpy as np
import json
import itertools

# Load the cosine similarity matrix and labels
cosine_sim_matrix = np.load("cosine_similarity_matrix.npy")

with open("domain_labels.json", "r") as f:
    labels = json.load(f)

# Remove the last row and column
reduced_matrix = cosine_sim_matrix[:-1, :-1]
print(labels)
reduced_labels = labels[:-1]
print(reduced_labels)

# Generate all k=5 combinations
combinations = list(itertools.combinations(range(len(reduced_labels)), 3))

# Compute the sum for each combination
combo_sums = []
for combo in combinations:
    submatrix = reduced_matrix[np.ix_(combo, combo)]
    combo_sum = np.sum(submatrix)
    combo_sums.append((combo, combo_sum))

# Sort by sum
combo_sums.sort(key=lambda x: x[1])

# Pick 5 spread-out examples
total_combos = len(combo_sums)
spread_indices = [0, total_combos // 4, total_combos // 2, 3 * total_combos // 4, total_combos - 1]
selected = [combo_sums[i] for i in spread_indices]

# Print results
for idx, (combo, sim_sum) in enumerate(selected):
    print(f"--- Combination {idx + 1} ---")
    print(f"Indices: {combo}")
    print(f"Labels: {[reduced_labels[i] for i in combo]}")
    print(f"Cosine similarity sum: {sim_sum:.4f}\n")


import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np

# Create a 1-row, 5-column figure
fig, axes = plt.subplots(1, 5, figsize=(22, 5))

for idx, (combo, sim_sum) in enumerate(selected):
    submatrix = reduced_matrix[np.ix_(combo, combo)]
    combo_labels = [reduced_labels[i] for i in combo]

    # Upper triangle mask
    mask = np.tril(np.ones_like(submatrix, dtype=bool))

    ax = axes[idx]
    sns.heatmap(
        submatrix,
        mask=mask,
        xticklabels=combo_labels,
        yticklabels=combo_labels,
        annot=True,
        fmt=".2f",
        cmap="coolwarm",
        linewidths=0.5,
        cbar=False,
        ax=ax,
        vmin=0,
        vmax=1,  # Fix color scale from 0 to 1
    )
    ax.set_title(f"Combo {idx + 1}\nSum: {sim_sum:.2f}", fontsize=10)
    ax.tick_params(axis='x', rotation=0)  # No rotation
    ax.tick_params(axis='y', rotation=0)

plt.tight_layout()
plt.show()
