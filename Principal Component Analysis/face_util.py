import matplotlib.pyplot as plt

def plot_different_pcs(X, num_pcs, imsize, X_mean, X_std, principal_components):
    fig, axes = plt.subplots(1, len(num_pcs)+1, figsize=(10, 5))
    axes[0].imshow(X.reshape(imsize), cmap='gray')
    axes[0].axis('off')
    axes[0].set_title('Original')
    b = (X - X_mean) / X_std
    
    for i, num_pc in enumerate(num_pcs, 1):
        u = principal_components[:, :num_pc]
        coefficients = b @ u

        b_recon = coefficients @ u.T
        X_recon = b_recon * X_std + X_mean
        
        axes[i].imshow(X_recon.reshape(imsize), cmap='gray')
        axes[i].axis('off')
        axes[i].set_title(f'$k={num_pc}$', fontdict={"fontsize": 16})
    plt.show()

def map_labels(labels):
    """Dataset's label come in string of emotions... 
    We want to use numbers so we can"""
    mapping = {'anger': 0, 'contempt': 1, 'disgust': 2, 'fear': 3, 'happy': 4, 'sadness': 5, 'surprise': 6}
    return [mapping[lbl] for lbl in labels]