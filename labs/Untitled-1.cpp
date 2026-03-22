const NUM_PARTICLES = 6000;        // cantidad de particulas
    const PARTICLE_MAX_LIFE = 30;     // vida maxima en frames
    const FADE_FRAMES = 10;            // frames de fade-in y fade-out
    const FLOW_SPEED_BASE = 2;       // velocidad base de flujo
    const FLOW_SPEED_MULT = 2.5;       // multiplicador de velocidad por distancia
    const STROKE_BASE = 2;            // tamaño base de particula (px)
    const STROKE_MULT = 5;            // multiplicador de tamaño por temperatura
    const BG_FADE_BASE = 4;            // alpha base del fondo (trail persistence)
    const BG_FADE_MULT = 2;           // multiplicador de fade por humedad
    const MAX_DRIFT_BASE = 15;         // radio minimo de deriva (px)
    const MAX_DRIFT_MULT = 25;         // radio extra de deriva por distancia
    const FLOW_NOISE_SCALE = 0.015;    // escala del Perlin noise (menor = mas suave)
    const HOME_PULL_STRENGTH = 0.08;   // fuerza de retorno al hogar
    const COLOR_TINT = 0.1;  