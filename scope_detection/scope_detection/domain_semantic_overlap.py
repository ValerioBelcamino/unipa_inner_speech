from sentence_transformers import SentenceTransformer
from sklearn.metrics.pairwise import cosine_similarity
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
import evaluate

from domain_examples.en import domain_descriptions



# Extract domain labels
labels = [desc.split(":")[0] for desc in domain_descriptions]

# Load models
model = SentenceTransformer('all-mpnet-base-v2')    #  all-mpnet-base-v2, all-MiniLM-L6-v2, paraphrase-multilingual-MiniLM-L12-v2
# bertscore = evaluate.load("bertscore")

# Embed descriptions
embeddings = model.encode(domain_descriptions)

# Compute cosine similarity matrix
cosine_sim_matrix = cosine_similarity(embeddings)

# # Prepare BERTScore matrix
# n = len(domain_descriptions)
# bertscore_f1_matrix = np.zeros((n, n))

# def compute_metrics(prediction: str, reference: str):
#     bertscore_result = bertscore.compute(predictions=[prediction], references=[reference], lang="en")
#     bert_f1 = bertscore_result["f1"][0]
#     return bert_f1

# # Compute pairwise BERTScore F1 matrix (only upper triangle and diagonal)
# for i in range(n):
#     for j in range(i, n):
#         f1_score = compute_metrics(domain_descriptions[i], domain_descriptions[j])
#         bertscore_f1_matrix[i, j] = f1_score
#         bertscore_f1_matrix[j, i] = f1_score  # symmetric

# Plot cosine similarity heatmap (triangular)
plt.figure(figsize=(12, 10))
mask = np.tril(np.ones_like(cosine_sim_matrix, dtype=bool))
sns.heatmap(cosine_sim_matrix, xticklabels=labels, yticklabels=labels, mask=mask,
            cmap="coolwarm", annot=True, fmt=".2f", linewidths=0.5)
plt.title("Cosine Similarity Between Domain Descriptions")
plt.tight_layout()
plt.show()

# # Plot BERTScore F1 heatmap (triangular)
# plt.figure(figsize=(12, 10))
# mask = np.tril(np.ones_like(bertscore_f1_matrix, dtype=bool))
# sns.heatmap(bertscore_f1_matrix, xticklabels=labels, yticklabels=labels, mask=mask,
#             cmap="coolwarm", annot=True, fmt=".2f", linewidths=0.5)
# plt.title("BERTScore F1 Similarity Between Domain Descriptions")
# plt.tight_layout()
# plt.show()

# Print top correlated pairs for both metrics
def print_top_pairs(matrix, metric_name, labels, top_k=5):
    n = matrix.shape[0]
    pairs = []
    for i in range(n):
        for j in range(i+1, n):
            pairs.append(((labels[i], labels[j]), matrix[i, j]))
    pairs.sort(key=lambda x: x[1], reverse=True)
    print(f"\nTop {top_k} pairs by {metric_name}:")
    for (label1, label2), score in pairs[:top_k]:
        print(f"{label1} <-> {label2}: {score:.4f}")

print_top_pairs(cosine_sim_matrix, "Cosine Similarity", labels)
# print_top_pairs(bertscore_f1_matrix, "BERTScore F1", labels)