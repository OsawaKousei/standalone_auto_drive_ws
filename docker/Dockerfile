FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# 基本ツールのインストール
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    gcc-12 \
    g++-12 \
    git \
    ca-certificates \
    curl \
    wget \
    gnupg \
    lsb-release \
    software-properties-common \
    libx11-6 \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# LLVM公式リポジトリから Clang 17 をインストール
# (Ubuntu 22.04標準のClang 14はGCC12のrangesヘッダーと相性が悪いため)
RUN wget https://apt.llvm.org/llvm.sh && \
    chmod +x llvm.sh && \
    ./llvm.sh 17 && \
    rm llvm.sh && \
    apt-get install -y clang-tidy-17 clang-format-17

# update-alternatives でシステム標準のコマンドとして登録
RUN update-alternatives --install /usr/bin/clang clang /usr/bin/clang-17 100 && \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-17 100 && \
    update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-17 100 && \
    update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-17 100

# 3. uv のインストール (公式推奨のマルチステージビルド形式でクリーンに取得)
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

# ビルドには引き続き GCC 12 を使用
ENV CC=gcc-12 \
    CXX=g++-12 \
    CXXFLAGS="-std=c++20"

WORKDIR /workspace