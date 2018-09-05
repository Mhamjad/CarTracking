"""VOC Dataset Classes

Original author: Francisco Massa
https://github.com/fmassa/vision/blob/voc_dataset/torchvision/datasets/voc.py

Updated by: Ellis Brown, Max deGroot
"""

import os
import os.path
import sys
import torch
import torch.utils.data as data
import torchvision.transforms as transforms
from PIL import Image, ImageDraw, ImageFont
import cv2
import numpy as np
if sys.version_info[0] == 2:
    import xml.etree.cElementTree as ET
else:
    import xml.etree.ElementTree as ET



VOC_CLASSES = (  # always index 0
    'Car', )
# 'bicycle', 'bird', 'boat',
#     'bottle', 'bus', 'car', 'cat', 'chair',
#     'cow', 'diningtable', 'dog', 'horse',
#     'motorbike', 'person', 'pottedplant',
    # 'sheep', 'sofa', 'train', 'tvmonitor')

# for making bounding boxes pretty
COLORS = ((255, 0, 0, 128), (0, 255, 0, 128), (0, 0, 255, 128),
          (0, 255, 255, 128), (255, 0, 255, 128), (255, 255, 0, 128))

plot = False

if plot:
    import matplotlib.pyplot as plt

class AnnotationTransform(object):
    """Transforms a VOC annotation into a Tensor of bbox coords and label index
    Initilized with a dictionary lookup of classnames to indexes

    Arguments:
        class_to_ind (dict, optional): dictionary lookup of classnames -> indexes
            (default: alphabetic indexing of VOC's 20 classes)
        keep_difficult (bool, optional): keep difficult instances or not
            (default: False)
        height (int): height
        width (int): width
    """

    def __init__(self, class_to_ind=None, keep_difficult=False):
        self.class_to_ind = class_to_ind or dict(
            zip(VOC_CLASSES, range(len(VOC_CLASSES))))
        self.keep_difficult = keep_difficult

    def __call__(self, filepath, width, height):
        """
        Arguments:
            target (annotation) : the target annotation to be made usable
                will be an ET.Element
        Returns:
            a list containing lists of bounding boxes  [bbox coords, class name]
        """
        # res = []
        # for obj in target.iter('object'):
        #     difficult = int(obj.find('difficult').text) == 1
        #     if not self.keep_difficult and difficult:
        #         continue
        #     name = obj.find('name').text.lower().strip()
        #     bbox = obj.find('bndbox')

        #     pts = ['xmin', 'ymin', 'xmax', 'ymax']
        #     bndbox = []
        #     for i, pt in enumerate(pts):
        #         cur_pt = int(bbox.find(pt).text) - 1
        #         # scale height or width
        #         cur_pt = cur_pt / width if i % 2 == 0 else cur_pt / height
        #         bndbox.append(cur_pt)
        #     label_idx = self.class_to_ind[name]
        #     bndbox.append(label_idx)
        #     res += [bndbox]  # [xmin, ymin, xmax, ymax, label_ind]
        #     # img_id = target.find('filename').text[:-4]

        # return res  # [[xmin, ymin, xmax, ymax, label_ind], ... ]

        obj = []
        objects = []

        with open(filepath, 'r') as f:
            for line in f:
                obj = line.split(' ')

                name = obj[0]
                if (name in VOC_CLASSES):
                    label_idx = self.class_to_ind[name]
                    xmin = float(obj[1])
                    ymin = float(obj[2])
                    xmax = float(obj[3])
                    ymax = float(obj[4])

                    if (plot):
                        plt.plot([xmin, xmax, xmax, xmin, xmin], [ymin, ymin, ymax, ymax, ymin], linewidth=3)
                        plt.text(xmin, ymin, name, fontsize=6, bbox=dict(facecolor='magenta', alpha=0.5), color='w')

                    objParsed = [xmin/width, ymin/height, xmax/width, ymax/height, label_idx]

                    objects += [objParsed]
                # else:
                #     objParsed = [0, 0, 0, 0, 4]


        if (plot):
            plt.show()

        return objects # [[xmin, ymin, xmax, ymax, label_ind], ... ]




class VOCDetection(data.Dataset):
    """VOC Detection Dataset Object

    input is image, target is annotation

    Arguments:
        root (string): filepath to VOCdevkit folder.
        image_set (string): imageset to use (eg. 'train', 'val', 'test')
        transform (callable, optional): transformation to perform on the
            input image
        target_transform (callable, optional): transformation to perform on the
            target `annotation`
            (eg: take in caption string, return tensor of word indices)
        dataset_name (string, optional): which dataset to load
            (default: 'VOC2007')
    """

    def __init__(self, root, image_sets, transform=None, target_transform=None,
                 dataset_name='VOC0712'):
        self.root = root
        self.image_set = image_sets
        self.transform = transform
        self.target_transform = target_transform
        self.name = dataset_name
        self._annopath = os.path.join('%s', 'labels', '%s.txt')
        self._imgpath = os.path.join('%s', 'images_test3', 'captured124-027_%s.jpg')
        self.ids = list()

        rootpath = os.path.join(self.root)
        for line in open(os.path.join(rootpath, self.image_set + '.txt')):
            self.ids.append((rootpath, line.strip()))

    def __getitem__(self, index):
        im, gt, h, w = self.pull_item(index)

        return im, gt

    def __len__(self):
        return len(self.ids)

    def pull_item(self, index):
        img_id = self.ids[index]

        img_path = self._annopath % img_id
        # target = ET.parse(img_path).getroot()
        img = cv2.imread(self._imgpath % img_id)
        height, width, channels = img.shape

        if plot:
            plt.imshow(img)

        if self.target_transform is not None:
            target = self.target_transform(img_path, width, height)

        if self.transform is not None:
            target = np.array(target)
            img, boxes, labels = self.transform(img, target[:, :4], target[:, 4])
            # to rgb
            img = img[:, :, (2, 1, 0)]
            # img = img.transpose(2, 0, 1)
            target = np.hstack((boxes, np.expand_dims(labels, axis=1)))
        return torch.from_numpy(img).permute(2, 0, 1), target, height, width
        # return torch.from_numpy(img), target, height, width

    def pull_image(self, index):
        '''Returns the original image object at index in PIL form

        Note: not using self.__getitem__(), as any transformations passed in
        could mess up this functionality.

        Argument:
            index (int): index of img to show
        Return:
            PIL img
        '''
        img_id = self.ids[index]
        return cv2.imread(self._imgpath % img_id, cv2.IMREAD_COLOR)

    def pull_anno(self, index):
        '''Returns the original annotation of image at index

        Note: not using self.__getitem__(), as any transformations passed in
        could mess up this functionality.

        Argument:
            index (int): index of img to get annotation of
        Return:
            list:  [img_id, [(label, bbox coords),...]]
                eg: ('001718', [('dog', (96, 13, 438, 332))])
        '''
        img_id = self.ids[index]
        anno = self._annopath % img_id
        # anno = ET.parse(self._annopath % img_id).getroot()
        gt = self.target_transform(anno, 1, 1)
        return img_id[1], gt

    def pull_tensor(self, index):
        '''Returns the original image at an index in tensor form

        Note: not using self.__getitem__(), as any transformations passed in
        could mess up this functionality.

        Argument:
            index (int): index of img to show
        Return:
            tensorized version of img, squeezed
        '''
        return torch.Tensor(self.pull_image(index)).unsqueeze_(0)


def detection_collate(batch):
    """Custom collate fn for dealing with batches of images that have a different
    number of associated object annotations (bounding boxes).

    Arguments:
        batch: (tuple) A tuple of tensor images and lists of annotations

    Return:
        A tuple containing:
            1) (tensor) batch of images stacked on their 0 dim
            2) (list of tensors) annotations for a given image are stacked on 0 dim
    """
    targets = []
    imgs = []
    for sample in batch:
        imgs.append(sample[0])
        targets.append(torch.FloatTensor(sample[1]))
    return torch.stack(imgs, 0), targets
