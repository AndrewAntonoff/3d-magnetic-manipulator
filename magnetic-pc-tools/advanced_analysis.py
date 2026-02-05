# advanced_analysis.py
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import seaborn as sns

def analyze_magnetic_data(filename):
    """Анализ сохраненных данных"""
    # Загрузка данных
    df = pd.read_csv(filename)
    
    # Основные статистики
    print("=== Magnetic Field Analysis ===")
    print(f"Total samples: {len(df)}")
    print(f"Duration: {df['time_s'].max():.1f} seconds")
    print(f"Sampling rate: {len(df)/df['time_s'].max():.1f} Hz")
    print("\nField Statistics (µT):")
    print(df[['x_ut', 'y_ut', 'z_ut', 'magnitude']].describe())
    
    # Графики
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    
    # Временные ряды
    axes[0, 0].plot(df['time_s'], df['x_ut'], label='X', alpha=0.7)
    axes[0, 0].plot(df['time_s'], df['y_ut'], label='Y', alpha=0.7)
    axes[0, 0].plot(df['time_s'], df['z_ut'], label='Z', alpha=0.7)
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Field (µT)')
    axes[0, 0].set_title('Magnetic Field Components')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Магнитуда
    axes[0, 1].plot(df['time_s'], df['magnitude'], 'r-', linewidth=2)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Magnitude (µT)')
    axes[0, 1].set_title('Total Magnetic Field Strength')
    axes[0, 1].grid(True, alpha=0.3)
    
    # Гистограммы
    axes[1, 0].hist(df['x_ut'], bins=50, alpha=0.7, label='X')
    axes[1, 1].hist(df['y_ut'], bins=50, alpha=0.7, label='Y')
    axes[2, 0].hist(df['z_ut'], bins=50, alpha=0.7, label='Z')
    axes[2, 1].hist(df['magnitude'], bins=50, alpha=0.7, label='Magnitude', color='red')
    
    for ax, title in zip([axes[1, 0], axes[1, 1], axes[2, 0], axes[2, 1]], 
                        ['X Distribution', 'Y Distribution', 'Z Distribution', 'Magnitude Distribution']):
        ax.set_xlabel('Field (µT)')
        ax.set_ylabel('Frequency')
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Спектральный анализ
    if len(df) > 100:
        fs = len(df) / df['time_s'].max()  # Частота дискретизации
        
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        for idx, (col, title) in enumerate(zip(['x_ut', 'y_ut', 'z_ut', 'magnitude'], 
                                               ['X Spectrum', 'Y Spectrum', 'Z Spectrum', 'Magnitude Spectrum'])):
            ax = axes[idx//2, idx%2]
            f, Pxx = signal.welch(df[col], fs, nperseg=min(256, len(df)//4))
            ax.semilogy(f, Pxx)
            ax.set_xlabel('Frequency (Hz)')
            ax.set_ylabel('Power Spectral Density')
            ax.set_title(title)
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()

# Использование
if __name__ == "__main__":
    analyze_magnetic_data('magnetic_data_20260130_112401.csv')