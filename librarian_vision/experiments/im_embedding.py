#!/usr/bin/env python3

#%%
from PIL import Image
import torch
from transformers import CLIPProcessor, CLIPModel

model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32")
processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")

#%%


def get_embedding(image):
    """ Turns the given image into a ambedding vector of length 512. """
    mapping = processor.feature_extractor(image)
    tensor_p_values = torch.Tensor(mapping['pixel_values'])

    img_pixel = {"pixel_values": tensor_p_values}

    with torch.no_grad():
        query_embeddings = model.get_image_features(**img_pixel)
        return query_embeddings

def compute_scores(emb_one, emb_two):
    """Computes cosine similarity between two vectors."""

    return torch.nn.functional.cosine_similarity(emb_one, emb_two)

img1 = Image.open("book_blue.png")
emb1 = get_embedding(img1)

img2 = Image.open("book_yellow.png")
emb2 = get_embedding(img2)
# print(emb1.shape)

compute_scores(emb1,emb2)