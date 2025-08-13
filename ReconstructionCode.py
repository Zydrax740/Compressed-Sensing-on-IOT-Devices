import numpy as np
import matplotlib.pyplot as plt
import cv2
import serial
import time
import pywt

# ===================== CONFIG =====================
image_shape = (96, 96)
image_size = image_shape[0] * image_shape[1]
threshold= 0.5

threshold_int32 = np.float32(threshold)

serial_port = 'COM10'
baudrate = 115200
timeout_seconds = 15

# ===================== CHAOTIC MASK =====================
def signum_map(size, seed=0.7, r=1.99):

    x = np.float32(seed)
    nr = np.float32(r)

    seq = []
    for i in range(size):
        x = x / abs(x) - (nr * x)
        x = np.float32(x)
        seq.append(x)

    chaotic_seq = np.array(seq, dtype=np.float32)
    return chaotic_seq

# ===================== READ PIXEL VALUES =====================
def read_values_only_from_serial(port='COM10', baudrate=115200, timeout_seconds=15):
    values = []
    start_time = time.time()
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Reading data from {port}...")

        while True:
            line = ser.readline().decode(errors='ignore').strip()
            start_time = time.time() 

            if line:
                data = line.split()
                print(f"Data: {data}")

                data = data[1:]

                for i in range(len(data) - 1):
                    if data[i].upper() == 'FF' and data[i + 1] == '00':
                        print("🔚 End marker (FF 00) received. Stopping.")
                        ser.close()
                        print(f"✅ Received {len(values)} non-zero pixel values.")
                        return values

                for d in data:
                    try:
                        values.append(int(d, 16))
                    except ValueError:
                        print(f"⚠️ Invalid hex value: {d}")

            if time.time() - start_time > timeout_seconds:
                print(f"⏱️ Timeout reached. No new data.")
                break

        ser.close()
    except Exception as e:
        print(f"❌ Serial error: {e}")
    print(f"✅ Received {len(values)} non-zero pixel values.")
    return values

# ===================== SPARSE IMAGE RECONSTRUCTION =====================
def rebuild_sparse_image(pixel_values, chaotic_seq, threshold_int32=0.5):

    mask = (chaotic_seq > threshold_int32) 
    print(mask[:20])

    sparse_image = np.zeros(image_size, dtype=int)

    non_zero_indices = np.where(mask == 1)[0]  

    for i, value in enumerate(pixel_values):
        if i < len(non_zero_indices):
            index = non_zero_indices[i]  
            sparse_image[index] = value  

    non_zero_count = np.count_nonzero(sparse_image)
    zero_count = sparse_image.size - non_zero_count

    print(f"Number of non-zero pixels in the sparse image: {non_zero_count}")
    print(f"Number of zero pixels in the sparse image: {zero_count}")

    return sparse_image.reshape(image_shape)

# ===================== WAVELET RECONSTRUCTION =====================
def soft_thresh(x, lam):
    return np.sign(x) * np.maximum(np.abs(x) - lam, 0)

def flat_wavelet_transform2(x, method='haar'):
    coeffs = pywt.wavedec2(x, method, level=2)
    flat, slices = pywt.coeffs_to_array(coeffs)
    return flat, slices

def inverse_flat_wavelet_transform2(flat, slices, method='haar'):
    coeffs_from_array = pywt.array_to_coeffs(flat, slices, output_format='wavedec2')
    return pywt.waverec2(coeffs_from_array, method)

def wavelet_reconstruction(sparse_image, max_iterations=200, lam=0.1, lam_decay=0.98):
    mse_per_iteration = []
    xhat = sparse_image.copy()
    for _ in range(max_iterations):
        flat, slices = flat_wavelet_transform2(xhat)
        flat_thresh = soft_thresh(flat, lam)
        xhat = inverse_flat_wavelet_transform2(flat_thresh, slices)
        mse = np.mean((xhat - sparse_image) ** 2)
        mse_per_iteration.append(mse)
        lam *= lam_decay
    return xhat, mse_per_iteration

# ===================== VISUALIZE =====================
def visualize_image(image, title):
    plt.imshow(image, cmap='gray')
    plt.title(title)
    plt.axis('off')
    plt.show()

# ===================== MAIN =====================
if __name__ == "__main__":

    image32 = np.int32(image_size)

    chaotic_seq = signum_map(image32) 

    pixel_values = read_values_only_from_serial(serial_port, baudrate, timeout_seconds)

    pixel_values_32 = np.int32(pixel_values)

    sparse_image = rebuild_sparse_image(pixel_values_32, chaotic_seq, threshold)

    visualize_image(sparse_image, "Compressed Sparse Image")

    reconstructed_image, mse_curve = wavelet_reconstruction(sparse_image)

    plt.plot(mse_curve)
    plt.title("MSE over Iterations")
    plt.xlabel("Iteration")
    plt.ylabel("MSE")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    reconstructed_image = np.clip(reconstructed_image, 0, 1)
    visualize_image(reconstructed_image, "Final Reconstructed Image")

    output_file = "./reconstructed_image.jpg"
    cv2.imwrite(output_file, (reconstructed_image * 255).astype(np.uint8))
    print(f"💾 Image saved to: {output_file}")
