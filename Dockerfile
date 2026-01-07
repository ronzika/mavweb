# docker run --rm -p 8501:8501 -p 14550:14550/udp mavweb:localdocker run --rm -p 8501:8501 -p 14550:14550/udp mavweb:localdocker run --rm -p 8501:8501 -p 14551:14551/udp mavweb:local

FROM python:3.11-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    STREAMLIT_BROWSER_GATHER_USAGE_STATS=false \
    STREAMLIT_SERVER_HEADLESS=true \
    STREAMLIT_SERVER_ADDRESS=0.0.0.0 \
    STREAMLIT_SERVER_PORT=8501

WORKDIR /app

# System deps for common Python wheels (pandas/numpy), Pillow, and potential xml deps.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        gcc \
        g++ \
        libjpeg62-turbo-dev \
        zlib1g-dev \
        libxml2-dev \
        libxslt1-dev \
    && rm -rf /var/lib/apt/lists/*

COPY requirements*.txt ./

RUN pip install --upgrade pip \
    && pip install -r requirements.txt \
    && if [ -f requirements-dashboard.txt ]; then pip install -r requirements-dashboard.txt; fi

COPY . .

EXPOSE 8501

CMD ["python", "-m", "streamlit", "run", "app.py", "--server.headless=true", "--server.port=8501", "--server.address=0.0.0.0"]
