CREATE TABLE arduino_nano_data (
    id              BIGSERIAL PRIMARY KEY,
    ts              TIMESTAMPTZ NOT NULL DEFAULT now(),

    -- Gas sensors
    mq_ratio        FLOAT4,
    mhmq_ratio      FLOAT4,

    -- Battery voltages
    battery_24v_v   FLOAT4,
    buck_19v_v      FLOAT4,

    -- Battery discharge currents
    battery_40v_a   FLOAT4,
    battery_24v_a   FLOAT4,

    -- Charger adapter currents
    charger_40v_a   FLOAT4,
    charger_24v_a   FLOAT4
);

-- Index on timestamp for fast time-range queries (dashboard/viz)
CREATE INDEX idx_arduino_nano_data_ts ON arduino_nano_data (ts DESC);
