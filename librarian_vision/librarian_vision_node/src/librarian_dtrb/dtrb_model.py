#!/usr/bin/env python3

from random import shuffle
import torch
import torch.backends.cudnn as cudnn
from torch.utils.data import Dataset, DataLoader
import torch.nn.functional as F
from librarian_dtrb.dataset import AlignCollate
from librarian_dtrb.utils import CTCLabelConverter, AttnLabelConverter
from librarian_dtrb.model import Model
from PIL import Image


class Namespace:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class InferDataset(Dataset):
    def __init__(self, images):
        self.images = images

    def __len__(self):
        return len(self.images)

    def __getitem__(self, index):
        return self.images[index]


class DTRB:
    def __init__(self, weights):
        self.device = torch.device(
            'cuda' if torch.cuda.is_available() else 'cpu')
        self.opt = Namespace(image_folder='.',
                             workers=1,
                             batch_size=192,
                             saved_model=weights,
                             batch_max_length=25,
                             imgH=32,
                             imgW=100,
                             character='0123456789abcdefghijklmnopqrstuvwxyz',
                             Transformation='TPS',
                             FeatureExtraction='ResNet',
                             SequenceModeling='BiLSTM',
                             Prediction='Attn',
                             num_fiducial=20,
                             input_channel=1,
                             output_channel=512,
                             hidden_size=256,
                             PAD=True,
                             num_gpu=1)

        self.converter = AttnLabelConverter(self.opt.character)
        self.opt.num_class = len(self.converter.character)
        self.model = Model(self.opt)
        self.model = torch.nn.DataParallel(self.model).to(self.device)
        self.model.load_state_dict(torch.load(
            self.opt.saved_model, map_location=self.device))
        self.model.eval()

    def infer(self, images):
        images = [Image.fromarray(im).convert('L') for im in images]
        data = InferDataset(images)
        align_collate = AlignCollate(
            imgH=self.opt.imgH, imgW=self.opt.imgW, keep_ratio_with_pad=self.opt.PAD)
        self.loader = DataLoader(
            data, batch_size=self.opt.batch_size, shuffle=False, collate_fn=align_collate, pin_memory=True
        )

        results = []
        with torch.no_grad():
            for image_tensors in self.loader:
                batch_size = image_tensors.size(0)
                image = image_tensors.to(self.device)
                # For max length prediction
                length_for_pred = torch.IntTensor([self.opt.batch_max_length] * batch_size).to(self.device)
                text_for_pred = torch.LongTensor(batch_size, self.opt.batch_max_length + 1).fill_(0).to(self.device)

                if 'CTC' in self.opt.Prediction:
                    preds = self.model(image, text_for_pred)
                    # Select max probabilty (greedy decoding) then decode index to character
                    preds_size = torch.IntTensor([preds.size(1)] * batch_size)
                    _, preds_index = preds.max(2)
                    # preds_index = preds_index.view(-1)
                    preds_str = self.converter.decode(preds_index, preds_size)
                else:
                    preds = self.model(image, text_for_pred, is_train=False)
                    # select max probabilty (greedy decoding) then decode index to character
                    _, preds_index = preds.max(2)
                    preds_str = self.converter.decode(preds_index, length_for_pred)
                
                preds_prob = F.softmax(preds, dim=2)
                preds_max_prob, _ = preds_prob.max(dim=2)

                for pred, max_prob in zip(preds_str, preds_max_prob):
                    pred_EOS = pred.find('[s]')
                    pred = pred[:pred_EOS]
                    max_prob = max_prob[:pred_EOS]
                    confidence = max_prob.cumprod(dim=0)[-1]
                    results.append((pred, confidence.item()))

        return results


# files = [
#     'test-crop-0.png',
#     'test-crop-1.png',
#     'test-crop-2.png'
# ]
# images = [Image.open(f) for f in files]
# model = DTRB()
# results = model.infer(images)
# print(results)
