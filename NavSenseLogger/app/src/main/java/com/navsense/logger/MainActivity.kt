    package com.navsense.logger

    import android.Manifest
    import android.content.pm.PackageManager
    import android.graphics.ImageFormat
    import android.os.Bundle
    import android.os.SystemClock
    import android.util.Log
    import android.widget.Button
    import android.widget.EditText
    import android.widget.TextView
    import androidx.appcompat.app.AppCompatActivity
    import androidx.camera.core.CameraSelector
    import androidx.camera.core.ImageAnalysis
    import androidx.camera.core.ImageProxy
    import androidx.camera.core.Preview
    import androidx.camera.lifecycle.ProcessCameraProvider
    import androidx.camera.view.PreviewView
    import androidx.core.app.ActivityCompat
    import androidx.core.content.ContextCompat
    import java.io.BufferedInputStream
    import java.io.BufferedOutputStream
    import java.io.DataInputStream
    import java.io.DataOutputStream
    import java.io.IOException
    import java.net.InetSocketAddress
    import java.net.Socket
    import java.nio.ByteBuffer
    import java.nio.ByteOrder
    import java.util.concurrent.ExecutorService
    import java.util.concurrent.Executors
    import java.util.concurrent.atomic.AtomicBoolean
    import kotlin.math.max

    class MainActivity : AppCompatActivity() {
        private lateinit var preview: PreviewView
        private lateinit var status: TextView
        private lateinit var host: EditText
        private lateinit var port: EditText
        private lateinit var start: Button
        private lateinit var stop: Button

        private val cameraExecutor: ExecutorService = Executors.newSingleThreadExecutor()
        private val sensorExecutor: ExecutorService = Executors.newSingleThreadExecutor()
        private var client: NavSenseClient? = null
        private var running = AtomicBoolean(false)
        private var cameraProvider: ProcessCameraProvider? = null

        private val sensorListener = object : android.hardware.SensorEventListener {
            override fun onSensorChanged(event: android.hardware.SensorEvent) {
                if (!running.get()) return
                val type = when (event.sensor.type) {
                    android.hardware.Sensor.TYPE_ACCELEROMETER -> "accelerometer"
                    android.hardware.Sensor.TYPE_GYROSCOPE -> "gyroscope"
                    android.hardware.Sensor.TYPE_MAGNETIC_FIELD -> "magnetometer"
                    else -> return
                }
                val values = event.values.joinToString(",") { it.toString() }
                val phoneWallMs = System.currentTimeMillis()
                val phoneMonoNs = SystemClock.elapsedRealtimeNanos()
                val json = """{"type":"imu","sensor":"$type","t_wall_ms":$phoneWallMs,"t_mono_ns":$phoneMonoNs,"values":[$values]}"""

                // Send on a background thread — this is the ONLY fix needed.
                val c = client ?: return
                sensorExecutor.execute {
                    try {
                        c.sendJson(json)
                    } catch (_: Exception) {
                    }
                }
            }
            override fun onAccuracyChanged(sensor: android.hardware.Sensor?, accuracy: Int) {}
        }

        override fun onCreate(savedInstanceState: Bundle?) {
            super.onCreate(savedInstanceState)
            setContentView(R.layout.activity_main)

            preview = findViewById(R.id.preview)
            status = findViewById(R.id.status)
            host = findViewById(R.id.host)
            port = findViewById(R.id.port)
            start = findViewById(R.id.start)
            stop = findViewById(R.id.stop)

            start.setOnClickListener { startLogging() }
            stop.setOnClickListener { stopLogging() }

            if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), 10)
            }

            setupSensors()
        }

        private fun setupSensors() {
            val sm = getSystemService(SENSOR_SERVICE) as android.hardware.SensorManager
            listOf(
                android.hardware.Sensor.TYPE_ACCELEROMETER,
                android.hardware.Sensor.TYPE_GYROSCOPE,
                android.hardware.Sensor.TYPE_MAGNETIC_FIELD
            ).forEach { t ->
                sm.getDefaultSensor(t)?.let {
                    sm.registerListener(sensorListener, it, android.hardware.SensorManager.SENSOR_DELAY_FASTEST)
                }
            }
        }

        private fun startLogging() {
            val h = host.text.toString().trim()
            val p = port.text.toString().toIntOrNull() ?: 8765
            if (h.isEmpty()) {
                status.text = "Enter laptop IP"
                return
            }

            running.set(true)
            start.isEnabled = false
            stop.isEnabled = true
            status.text = "Connecting..."

            client = NavSenseClient(h, p,
                onStatus = { s -> runOnUiThread { status.text = s } })
            client!!.connectAsync()

            startCamera()
        }

        private fun stopLogging() {
            running.set(false)
            client?.close()
            client = null
            start.isEnabled = true
            stop.isEnabled = false
            status.text = "Stopped"
        }

        private fun startCamera() {
            val future = ProcessCameraProvider.getInstance(this)
            future.addListener({
                cameraProvider = future.get()
                val provider = cameraProvider ?: return@addListener

                val previewUse = Preview.Builder().build().also {
                    it.surfaceProvider = preview.surfaceProvider
                }

                val analysis = ImageAnalysis.Builder()
                    .setTargetResolution(android.util.Size(640, 480))
                    .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                    .build()

                analysis.setAnalyzer(cameraExecutor) { image ->
                    if (running.get()) sendCamera(image)
                    image.close()
                }

                provider.unbindAll()
                provider.bindToLifecycle(
                    this,
                    CameraSelector.DEFAULT_BACK_CAMERA,
                    previewUse,
                    analysis
                )
            }, ContextCompat.getMainExecutor(this))
        }

        private fun sendCamera(image: ImageProxy) {
            val jpeg = yuv420ToJpeg(image, 40)
            val wall = System.currentTimeMillis()
            val mono = SystemClock.elapsedRealtimeNanos()
            client?.sendFrame(
                type = "camera",
                wallMs = wall,
                monoNs = mono,
                width = image.width,
                height = image.height,
                mime = "image/jpeg",
                payload = jpeg
            )
        }

        // Converts CameraX YUV_420_888 to JPEG without relying on device-specific NV21 assumptions.
        private fun yuv420ToJpeg(image: ImageProxy, quality: Int): ByteArray {
            val y = image.planes[0]
            val u = image.planes[1]
            val v = image.planes[2]
            val w = image.width
            val h = image.height

            val nv21 = ByteArray(w * h + 2 * (w / 2) * (h / 2))
            var pos = 0

            for (row in 0 until h) {
                val rowStart = row * y.rowStride
                for (col in 0 until w) {
                    nv21[pos++] = y.buffer.get(rowStart + col * y.pixelStride)
                }
            }
            for (row in 0 until h / 2) {
                val uRow = row * u.rowStride
                val vRow = row * v.rowStride
                for (col in 0 until w / 2) {
                    nv21[pos++] = v.buffer.get(vRow + col * v.pixelStride)
                    nv21[pos++] = u.buffer.get(uRow + col * u.pixelStride)
                }
            }

            val yuv = android.graphics.YuvImage(
                nv21, ImageFormat.NV21, w, h, null
            )
            val out = java.io.ByteArrayOutputStream()
            yuv.compressToJpeg(android.graphics.Rect(0, 0, w, h), quality, out)
            return out.toByteArray()
        }

        override fun onDestroy() {
            stopLogging()
            val sm = getSystemService(SENSOR_SERVICE) as android.hardware.SensorManager
            sm.unregisterListener(sensorListener)
            cameraExecutor.shutdown()
            sensorExecutor.shutdown()
            super.onDestroy()
        }
    }

    class NavSenseClient(
        private val host: String,
        private val port: Int,
        private val onStatus: (String) -> Unit
    ) {
        private var socket: Socket? = null
        private var out: DataOutputStream? = null
        private val lock = Any()
        private val executor = Executors.newSingleThreadExecutor()

        fun connectAsync() {
            executor.execute {
                try {
                    val s = Socket()
                    s.tcpNoDelay = true
                    s.connect(InetSocketAddress(host, port), 3000)
                    socket = s
                    out = DataOutputStream(BufferedOutputStream(s.getOutputStream(), 256 * 1024))
                    onStatus("Connected to $host:$port")
                    sendJson("""{"type":"hello","version":1,"phone_wall_ms":${System.currentTimeMillis()},"phone_mono_ns":${SystemClock.elapsedRealtimeNanos()}}""")
                    startReader(s)
                } catch (e: Exception) {
                    onStatus("Connection failed: ${e.message}")
                    close()
                }
            }
        }

        fun sendJson(json: String) {
            sendRecord(json.toByteArray(Charsets.UTF_8))
        }

        fun sendFrame(type: String, wallMs: Long, monoNs: Long, width: Int, height: Int, mime: String, payload: ByteArray) {
            val meta = """{"type":"$type","t_wall_ms":$wallMs,"t_mono_ns":$monoNs,"width":$width,"height":$height,"mime":"$mime","payload_len":${payload.size}}"""
            val metaBytes = meta.toByteArray(Charsets.UTF_8)
            val header = ByteBuffer.allocate(4).order(ByteOrder.BIG_ENDIAN).putInt(metaBytes.size).array()
            synchronized(lock) {
                try {
                    val o = out ?: return
                    o.write(header)
                    o.write(metaBytes)
                    o.write(payload)
                    o.flush()
                } catch (e: IOException) {
                    onStatus("Disconnected: ${e.message}")
                    close()
                }
            }
        }

        private fun startReader(s: Socket) {
            executor.execute {
                try {
                    val input = DataInputStream(BufferedInputStream(s.getInputStream(), 64 * 1024))
                    while (!s.isClosed) {
                        val n = input.readInt()
                        if (n <= 0 || n > 1024 * 1024) throw IOException("invalid command size: $n")
                        val data = ByteArray(n)
                        input.readFully(data)
                        val msg = String(data, Charsets.UTF_8)
                        if (msg.contains("\"type\":\"sync_req\"")) {
                            val recv = System.currentTimeMillis()
                            val mono = SystemClock.elapsedRealtimeNanos()
                            sendJson("""{"type":"sync_resp","phone_recv_wall_ms":$recv,"phone_recv_mono_ns":$mono,"phone_send_wall_ms":${System.currentTimeMillis()}}""")
                        }
                    }
                } catch (_: Exception) {
                    // Socket closure/disconnect is handled by the writer/status path.
                }
            }
        }

        private fun sendRecord(bytes: ByteArray) {
            synchronized(lock) {
                try {
                    val o = out ?: return
                    o.writeInt(bytes.size)
                    o.write(bytes)
                    o.flush()
                } catch (e: IOException) {
                    onStatus("Disconnected: ${e.message}")
                    close()
                }
            }
        }

        fun close() {
            try { socket?.close() } catch (_: Exception) {}
            socket = null
            out = null
        }
    }
