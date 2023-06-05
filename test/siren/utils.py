import matplotlib.pyplot as plt
import numpy as np
import torch
import dataio
import os
import diff_operators
from torchvision.utils import make_grid, save_image
import skimage.measure
import meta_modules
import scipy.io.wavfile as wavfile


def cond_mkdir(path):
    if not os.path.exists(path):
        os.makedirs(path)

def write_result_img(experiment_name, filename, img):
    root_path = '/media/data1/sitzmann/generalization/results'
    trgt_dir = os.path.join(root_path, experiment_name)

    img = img.detach().cpu().numpy()
    np.save(os.path.join(trgt_dir, filename), img)

def densely_sample_activations(model, num_dim=1, num_steps=int(1e6)):
    input = torch.linspace(-1., 1., steps=num_steps).float()

    if num_dim == 1:
        input = input[...,None]
    else:
        input = torch.stack(torch.meshgrid(*(input for _ in num_dim)), dim=-1).view(-1, num_dim)

    input = {'coords':input[None,:].cuda()}
    with torch.no_grad():
        activations = model.forward_with_activations(input)['activations']
    return activations

def make_contour_plot(array_2d,mode='log'):
    fig, ax = plt.subplots(figsize=(2.75, 2.75), dpi=300)

    if(mode=='log'):
        num_levels = 6
        levels_pos = np.logspace(-2, 0, num=num_levels) # logspace
        levels_neg = -1. * levels_pos[::-1]
        levels = np.concatenate((levels_neg, np.zeros((0)), levels_pos), axis=0)
        colors = plt.get_cmap("Spectral")(np.linspace(0., 1., num=num_levels*2+1))
    elif(mode=='lin'):
        num_levels = 10
        levels = np.linspace(-.5,.5,num=num_levels)
        colors = plt.get_cmap("Spectral")(np.linspace(0., 1., num=num_levels))

    sample = np.flipud(array_2d)
    CS = ax.contourf(sample, levels=levels, colors=colors)
    cbar = fig.colorbar(CS)

    ax.contour(sample, levels=levels, colors='k', linewidths=0.1)
    ax.contour(sample, levels=[0], colors='k', linewidths=0.3)
    ax.axis('off')
    return fig

def write_sdf_summary(model, model_input, gt, model_output, writer, total_steps, prefix='train_'):
    slice_coords_2d = dataio.get_mgrid(512)

    with torch.no_grad():
        yz_slice_coords = torch.cat((torch.zeros_like(slice_coords_2d[:, :1]), slice_coords_2d), dim=-1)
        yz_slice_model_input = {'coords': yz_slice_coords.cuda()[None, ...]}

        yz_model_out = model(yz_slice_model_input)
        sdf_values = yz_model_out['model_out']
        sdf_values = dataio.lin2img(sdf_values).squeeze().cpu().numpy()
        fig = make_contour_plot(sdf_values)
        writer.add_figure(prefix + 'yz_sdf_slice', fig, global_step=total_steps)

        xz_slice_coords = torch.cat((slice_coords_2d[:,:1],
                                     torch.zeros_like(slice_coords_2d[:, :1]),
                                     slice_coords_2d[:,-1:]), dim=-1)
        xz_slice_model_input = {'coords': xz_slice_coords.cuda()[None, ...]}

        xz_model_out = model(xz_slice_model_input)
        sdf_values = xz_model_out['model_out']
        sdf_values = dataio.lin2img(sdf_values).squeeze().cpu().numpy()
        fig = make_contour_plot(sdf_values)
        writer.add_figure(prefix + 'xz_sdf_slice', fig, global_step=total_steps)

        xy_slice_coords = torch.cat((slice_coords_2d[:,:2],
                                     -0.75*torch.ones_like(slice_coords_2d[:, :1])), dim=-1)
        xy_slice_model_input = {'coords': xy_slice_coords.cuda()[None, ...]}

        xy_model_out = model(xy_slice_model_input)
        sdf_values = xy_model_out['model_out']
        sdf_values = dataio.lin2img(sdf_values).squeeze().cpu().numpy()
        fig = make_contour_plot(sdf_values)
        writer.add_figure(prefix + 'xy_sdf_slice', fig, global_step=total_steps)

        min_max_summary(prefix + 'model_out_min_max', model_output['model_out'], writer, total_steps)
        min_max_summary(prefix + 'coords', model_input['coords'], writer, total_steps)


def hypernet_activation_summary(model, model_input, gt, model_output, writer, total_steps, prefix='train_'):
    with torch.no_grad():
        hypo_parameters, embedding = model.get_hypo_net_weights(model_input)

        for name, param in hypo_parameters.items():
            writer.add_histogram(prefix + name, param.cpu(), global_step=total_steps)

        writer.add_histogram(prefix + 'latent_code', embedding.cpu(), global_step=total_steps)


def write_gradcomp_summary(model, model_input, gt, model_output, writer, total_steps, prefix='train_'):
    # Plot gt gradients (this is what has been fitted)
    gt_gradients = gt['gradients']
    gt_grads_img = dataio.grads2img(dataio.lin2img(gt_gradients))

    pred_gradients = diff_operators.gradient(model_output['model_out'], model_output['model_in'])
    pred_grads_img = dataio.grads2img(dataio.lin2img(pred_gradients))

    output_vs_gt_gradients = torch.cat((gt_grads_img, pred_grads_img), dim=-1)
    writer.add_image(prefix + 'comp_gt_vs_pred_gradients', make_grid(output_vs_gt_gradients, scale_each=False, normalize=True),
                     global_step=total_steps)

    # Plot gt
    gt_grads1 = gt['grads1']
    gt_grads1_img = dataio.grads2img(dataio.lin2img(gt_grads1))

    gt_grads2 = gt['grads2']
    gt_grads2_img = dataio.grads2img(dataio.lin2img(gt_grads2))

    writer.add_image(prefix + 'gt_grads1', make_grid(gt_grads1_img, scale_each=False, normalize=True),
                     global_step=total_steps)
    writer.add_image(prefix + 'gt_grads2', make_grid(gt_grads2_img, scale_each=False, normalize=True),
                     global_step=total_steps)

    writer.add_image(prefix + 'gt_gradcomp', make_grid(gt_grads_img, scale_each=False, normalize=True),
                     global_step=total_steps)
    writer.add_image(prefix + 'pred_gradcomp', make_grid(pred_grads_img, scale_each=False, normalize=True),
                     global_step=total_steps)
    # Plot gt image
    gt_img1 = dataio.lin2img(gt['img1'])
    gt_img2 = dataio.lin2img(gt['img2'])
    writer.add_image(prefix + 'gt_img1', make_grid(gt_img1, scale_each=False, normalize=True),
                     global_step=total_steps)
    writer.add_image(prefix + 'gt_img2', make_grid(gt_img2, scale_each=False, normalize=True),
                     global_step=total_steps)

    # Plot pred compo image
    pred_img = dataio.rescale_img(dataio.lin2img(model_output['model_out']))
    writer.add_image(prefix + 'pred_comp_img', make_grid(pred_img, scale_each=False, normalize=True),
                     global_step=total_steps)

    min_max_summary(prefix + 'coords', model_input['coords'], writer, total_steps)
    min_max_summary(prefix + 'gt_laplace', gt_gradients, writer, total_steps)
    min_max_summary(prefix + 'pred_img', pred_img, writer, total_steps)


def min_max_summary(name, tensor, writer, total_steps):
    writer.add_scalar(name + '_min', tensor.min().detach().cpu().numpy(), total_steps)
    writer.add_scalar(name + '_max', tensor.max().detach().cpu().numpy(), total_steps)


def write_psnr(pred_img, gt_img, writer, iter, prefix):
    batch_size = pred_img.shape[0]

    pred_img = pred_img.detach().cpu().numpy()
    gt_img = gt_img.detach().cpu().numpy()

    psnrs, ssims = list(), list()
    for i in range(batch_size):
        p = pred_img[i].transpose(1, 2, 0)
        trgt = gt_img[i].transpose(1, 2, 0)

        p = (p / 2.) + 0.5
        p = np.clip(p, a_min=0., a_max=1.)

        trgt = (trgt / 2.) + 0.5

        ssim = skimage.measure.compare_ssim(p, trgt, multichannel=True, data_range=1)
        psnr = skimage.measure.compare_psnr(p, trgt, data_range=1)

        psnrs.append(psnr)
        ssims.append(ssim)

    writer.add_scalar(prefix + "psnr", np.mean(psnrs), iter)
    writer.add_scalar(prefix + "ssim", np.mean(ssims), iter)
