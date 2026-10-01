package com.navsense.logger

import android.Manifest
import android.annotation.SuppressLint
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.SurfaceTexture
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CameraMetadata
import android.hardware.camera2.CaptureRequest
import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaCodecList
import android.media.MediaFormat
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.os.SystemClock
import android.util.Range
import android.util.Size
import android.view.Surface
import android.view.TextureView
import android.view.WindowManager
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import java.io.BufferedOutputStream
import java.io.DataInputStream
import java.io.DataOutputStream
import java.io.IOException
import java.net.InetSocketAddress
import java.net.Socket
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicLong
import kotlin.math.abs

/** All tunables in one place. */
object Cfg {
    const val FPS = 30
    // Largest size the camera AND the encoder both support at 30 fps, up to this cap.
    // Raise to 3840/2160 to let the hardware decide; expect more latency + bandwidth.
    const val MAX_W = 1920
    const val MAX_H = 1080
    const val BITS_PER_PIXEL = 0.12          // 1080p30 -> ~7.5 Mbps
    const val ENABLE_PREVIEW = true          // set false to save power/CPU

    const val IMU_PERIOD_US = 5000           // 200 Hz per sensor (max without HIGH_SAMPLING_RATE_SENSORS)
    const val IMU_SAMPLES_PER_SENSOR_PER_PACKET = 2   // -> 100 packets/s
    const val SEND_MAGNETOMETER = false

    const val MAX_VIDEO_BACKLOG_BYTES = 2_000_000L    // beyond this, drop P-frames until next keyframe
}

class VideoPacket(
    val frameId: Long, val config: Boolean, val key: Boolean,
    val tCamNs: Long, val tEncNs: Long, val width: Int, val height: Int, val data: ByteArray
)

class MainActivity : AppCompatActivity() {
    private lateinit var preview: TextureView   // layout: change <PreviewView> to <TextureView>
    private lateinit var status: TextView
    private lateinit var host: EditText
    private lateinit var port: EditText
    private lateinit var start: Button
    private lateinit var stop: Button

    private var client: NavSenseClient? = null
    private var video: VideoStreamer? = null
    private var imu: ImuStreamer? = null
    private val ui = Handler(Looper.getMainLooper())
    @Volatile private var streaming = false
    private var lastFrames = 0L
    private var lastBytes = 0L

    private val ticker = object : Runnable {
        override fun run() {
            if (!streaming) return
            val f = video?.frames?.get() ?: 0L
            val b = client?.bytesSent?.get() ?: 0L
            status.text = "%s | %d fps | %.1f Mbps | dropped %d".format(
                video?.sizeText ?: "", f - lastFrames, (b - lastBytes) * 8 / 1e6, client?.dropped ?: 0L
            )
            lastFrames = f; lastBytes = b
            ui.postDelayed(this, 1000)
        }
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
        stop.setOnClickListener { stopLogging("Stopped") }
        stop.isEnabled = false
    }

    private fun startLogging() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), 10)
            status.text = "Grant camera permission, then press Start again"
            return
        }
        val h = host.text.toString().trim()
        val p = port.text.toString().toIntOrNull() ?: 8765
        if (h.isEmpty()) { status.text = "Enter laptop IP"; return }

        start.isEnabled = false
        stop.isEnabled = true
        status.text = "Connecting..."
        client = NavSenseClient(
            h, p,
            onStatus = { s -> runOnUiThread { status.text = s } },
            onStart = { runOnUiThread { beginStreaming() } },   // laptop says go after clock sync
            onLost = { runOnUiThread { stopLogging("Connection lost") } }
        ).also { it.start() }
    }

    private fun beginStreaming() {
        val c = client ?: return
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        val tex = if (Cfg.ENABLE_PREVIEW && preview.isAvailable) preview.surfaceTexture else null
        video = VideoStreamer(this, c, tex) { s -> runOnUiThread { status.text = s } }.also { it.start() }
        imu = ImuStreamer(getSystemService(SENSOR_SERVICE) as SensorManager, c).also { it.start() }
        lastFrames = 0; lastBytes = 0
        streaming = true
        ui.postDelayed(ticker, 1000)
    }

    private fun stopLogging(msg: String) {
        streaming = false
        ui.removeCallbacks(ticker)
        imu?.stop(); imu = null
        video?.stop(); video = null
        client?.close(); client = null
        window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        start.isEnabled = true
        stop.isEnabled = false
        status.text = msg
    }

    override fun onDestroy() {
        stopLogging("Stopped")
        super.onDestroy()
    }
}

/* ------------------------------------------------------------------------- */
/* IMU: sample at 200 Hz, ship 2 samples/sensor per packet (100 packets/s)    */
/* ------------------------------------------------------------------------- */
class ImuStreamer(private val sm: SensorManager, private val client: NavSenseClient) : SensorEventListener {
    private val thread = HandlerThread("imu").apply { start() }
    private val sb = StringBuilder(512)
    private var n = 0
    private var target = 4
    private var batchStartNs = 0L

    fun start() {
        val types = mutableListOf(Sensor.TYPE_ACCELEROMETER, Sensor.TYPE_GYROSCOPE)
        if (Cfg.SEND_MAGNETOMETER) types.add(Sensor.TYPE_MAGNETIC_FIELD)
        val h = Handler(thread.looper)
        var active = 0
        for (t in types) {
            val s = sm.getDefaultSensor(t) ?: continue
            if (sm.registerListener(this, s, Cfg.IMU_PERIOD_US, h)) active++
        }
        target = maxOf(1, active * Cfg.IMU_SAMPLES_PER_SENSOR_PER_PACKET)
    }

    fun stop() {
        sm.unregisterListener(this)
        thread.quitSafely()
    }

    // Runs only on the imu thread, so no locking needed.
    override fun onSensorChanged(e: SensorEvent) {
        val id = when (e.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> 0
            Sensor.TYPE_GYROSCOPE -> 1
            Sensor.TYPE_MAGNETIC_FIELD -> 2
            else -> return
        }
        if (n == 0) batchStartNs = e.timestamp else sb.append(',')
        sb.append('[').append(id).append(',').append(e.timestamp).append(',')
            .append(e.values[0]).append(',').append(e.values[1]).append(',').append(e.values[2]).append(']')
        n++
        // Flush on count, or after 20 ms so a stalled sensor can't hold samples back.
        if (n >= target || e.timestamp - batchStartNs > 20_000_000L) flush()
    }

    private fun flush() {
        client.sendJson(
            """{"type":"imu_batch","n":$n,"t_batch_ns":${SystemClock.elapsedRealtimeNanos()},"samples":[$sb]}"""
        )
        sb.setLength(0); n = 0
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
}

/* ------------------------------------------------------------------------- */
/* Video: Camera2 -> MediaCodec input Surface -> H.264 Annex-B packets        */
/* ------------------------------------------------------------------------- */
class VideoStreamer(
    private val ctx: Context,
    private val client: NavSenseClient,
    private val previewTexture: SurfaceTexture?,
    private val onStatus: (String) -> Unit
) {
    private val mime = MediaFormat.MIMETYPE_VIDEO_AVC
    private val thread = HandlerThread("camera").apply { start() }
    private val handler = Handler(thread.looper)

    val frames = AtomicLong(0)
    @Volatile var sizeText = ""

    private var camera: CameraDevice? = null
    private var session: CameraCaptureSession? = null
    private var codec: MediaCodec? = null
    private var encSurface: Surface? = null
    private var prevSurface: Surface? = null

    private lateinit var cm: CameraManager
    private lateinit var camId: String
    private lateinit var chars: CameraCharacteristics
    private lateinit var encInfo: MediaCodecInfo
    private lateinit var candidates: List<Size>
    private lateinit var fpsRange: Range<Int>
    private var tsSource = "unknown"
    private var size = Size(0, 0)
    private var bitrate = 0
    private var attempt = 0
    private var frameId = 0L
    private var camOffsetNs: Long? = null

    fun start() = handler.post {
        try {
            cm = ctx.getSystemService(Context.CAMERA_SERVICE) as CameraManager
            camId = cm.cameraIdList.first {
                cm.getCameraCharacteristics(it).get(CameraCharacteristics.LENS_FACING) == CameraCharacteristics.LENS_FACING_BACK
            }
            chars = cm.getCameraCharacteristics(camId)
            encInfo = pickEncoder() ?: throw IllegalStateException("no H.264 encoder")
            val vcaps = encInfo.getCapabilitiesForType(mime).videoCapabilities
            val map = chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)!!
            val frameNs = 1_000_000_000L / Cfg.FPS + 1_000_000L
            candidates = (map.getOutputSizes(MediaCodec::class.java) ?: emptyArray())
                .filter { it.width <= Cfg.MAX_W && it.height <= Cfg.MAX_H }
                .filter { vcaps.areSizeAndRateSupported(it.width, it.height, Cfg.FPS.toDouble()) }
                .filter { map.getOutputMinFrameDuration(MediaCodec::class.java, it) <= frameNs }
                .sortedByDescending { it.width * it.height }
                .take(4)
            if (candidates.isEmpty()) throw IllegalStateException("no size supports ${Cfg.FPS} fps")
            val ranges = chars.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES) ?: emptyArray()
            fpsRange = ranges.filter { it.upper == Cfg.FPS }.maxByOrNull { it.lower }
                ?: ranges.maxByOrNull { it.upper } ?: Range(Cfg.FPS, Cfg.FPS)
            tsSource = if (chars.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE) ==
                CameraMetadata.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME) "realtime" else "unknown"
            tryStart()
        } catch (e: Exception) {
            onStatus("Video start failed: ${e.message}")
        }
    }

    fun stop() {
        handler.post {
            cleanup()
            thread.quitSafely()
        }
    }

    private fun pickEncoder(): MediaCodecInfo? {
        val all = MediaCodecList(MediaCodecList.REGULAR_CODECS).codecInfos
            .filter { it.isEncoder && mime in it.supportedTypes }
        return all.firstOrNull { !it.name.startsWith("OMX.google.") && !it.name.startsWith("c2.android.") }
            ?: all.firstOrNull()
    }

    private fun toRealtimeNs(rawNs: Long): Long {
        var off = camOffsetNs
        if (off == null) {
            val rt = SystemClock.elapsedRealtimeNanos()
            val monoOff = rt - System.nanoTime()          // = time spent in deep sleep
            val base: String
            if (abs(rt - rawNs) < 1_000_000_000L) { off = 0L; base = "realtime" }
            else if (abs(rt - (rawNs + monoOff)) < 1_000_000_000L) { off = monoOff; base = "monotonic" }
            else { off = 0L; base = "unknown" }
            camOffsetNs = off
            client.sendJson("""{"type":"cam_clock","base":"$base","offset_ns":$off}""")
        }
        return rawNs + off
    }

    private fun tryStart() {
        if (attempt >= candidates.size) { onStatus("No working video configuration"); return }
        size = candidates[attempt]
        try {
            setupCodec(size)
        } catch (e: Exception) {
            onStatus("Encoder failed at ${size.width}x${size.height}: ${e.message}")
            releaseCodec(); attempt++; tryStart(); return
        }
        if (camera == null) openCamera() else beginSession()
    }

    private fun setupCodec(sz: Size) {
        val caps = encInfo.getCapabilitiesForType(mime)
        bitrate = caps.videoCapabilities.bitrateRange.clamp((sz.width * sz.height * Cfg.FPS * Cfg.BITS_PER_PIXEL).toInt())
        val cbr = caps.encoderCapabilities.isBitrateModeSupported(MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_CBR)
        val fmt = MediaFormat.createVideoFormat(mime, sz.width, sz.height).apply {
            setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface)
            setInteger(MediaFormat.KEY_BIT_RATE, bitrate)
            setInteger(MediaFormat.KEY_FRAME_RATE, Cfg.FPS)
            setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1)
            setInteger(MediaFormat.KEY_BITRATE_MODE,
                if (cbr) MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_CBR
                else MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_VBR)
            setInteger(MediaFormat.KEY_PRIORITY, 0)                       // realtime
            setInteger("prepend-sps-pps-to-idr-frames", 1)                // receiver can join at any keyframe
            if (Build.VERSION.SDK_INT >= 29) setInteger(MediaFormat.KEY_MAX_B_FRAMES, 0)
            if (Build.VERSION.SDK_INT >= 30) setInteger(MediaFormat.KEY_LATENCY, 1)
        }
        val c = MediaCodec.createByCodecName(encInfo.name)
        c.setCallback(codecCallback, handler)
        c.configure(fmt, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
        encSurface = c.createInputSurface()
        c.start()
        codec = c
    }

    private val codecCallback = object : MediaCodec.Callback() {
        override fun onInputBufferAvailable(c: MediaCodec, index: Int) {}
        override fun onOutputFormatChanged(c: MediaCodec, format: MediaFormat) {}
        override fun onError(c: MediaCodec, e: MediaCodec.CodecException) { onStatus("Encoder error: ${e.message}") }
        override fun onOutputBufferAvailable(c: MediaCodec, index: Int, info: MediaCodec.BufferInfo) {
            try {
                val buf = c.getOutputBuffer(index)
                if (buf != null && info.size > 0) {
                    val bytes = ByteArray(info.size)
                    buf.position(info.offset); buf.limit(info.offset + info.size); buf.get(bytes)
                    val config = info.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG != 0
                    val key = info.flags and MediaCodec.BUFFER_FLAG_KEY_FRAME != 0
                    if (!config) frames.incrementAndGet()
                    client.sendVideo(
                        VideoPacket(
                            if (config) -1 else frameId++, config, key,
                            toRealtimeNs(info.presentationTimeUs * 1000),   // camera capture time, now on elapsedRealtimeNanos base
                            SystemClock.elapsedRealtimeNanos(),       // encoder output time (ns)
                            size.width, size.height, bytes
                        )
                    )
                }
            } finally {
                c.releaseOutputBuffer(index, false)
            }
        }
    }

    @SuppressLint("MissingPermission")
    private fun openCamera() {
        cm.openCamera(camId, object : CameraDevice.StateCallback() {
            override fun onOpened(d: CameraDevice) { camera = d; beginSession() }
            override fun onDisconnected(d: CameraDevice) { d.close(); onStatus("Camera disconnected") }
            override fun onError(d: CameraDevice, error: Int) { d.close(); onStatus("Camera error $error") }
        }, handler)
    }

    @Suppress("DEPRECATION")
    private fun beginSession() {
        val cam = camera ?: return
        val enc = encSurface ?: return
        val map = chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)!!
        val surfaces = mutableListOf(enc)
        prevSurface?.release(); prevSurface = null
        if (previewTexture != null) {
            val pv = (map.getOutputSizes(SurfaceTexture::class.java) ?: emptyArray())
                .filter { it.width <= 1280 && it.width * size.height == it.height * size.width }
                .maxByOrNull { it.width * it.height } ?: Size(640, 480)
            previewTexture.setDefaultBufferSize(pv.width, pv.height)
            prevSurface = Surface(previewTexture).also { surfaces.add(it) }
        }
        try {
            cam.createCaptureSession(surfaces, object : CameraCaptureSession.StateCallback() {
                override fun onConfigured(s: CameraCaptureSession) {
                    session = s
                    try {
                        s.setRepeatingRequest(buildRequest(cam, surfaces).build(), null, handler)
                    } catch (e: Exception) { onStatus("Capture failed: ${e.message}"); return }
                    sizeText = "${size.width}x${size.height}"
                    client.sendJson(
                        """{"type":"video_info","codec":"h264","encoder":"${encInfo.name}","width":${size.width},"height":${size.height},"fps":${Cfg.FPS},"bitrate":$bitrate,"fps_range":"$fpsRange","ts_source":"$tsSource"}"""
                    )
                }
                override fun onConfigureFailed(s: CameraCaptureSession) {
                    onStatus("Session config failed at ${size.width}x${size.height}, trying smaller")
                    releaseCodec(); attempt++; tryStart()
                }
            }, handler)
        } catch (e: Exception) {
            onStatus("Session error: ${e.message}")
        }
    }

    private fun buildRequest(cam: CameraDevice, surfaces: List<Surface>): CaptureRequest.Builder {
        val b = cam.createCaptureRequest(CameraDevice.TEMPLATE_RECORD)
        surfaces.forEach { b.addTarget(it) }
        b.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, fpsRange)
        // Stabilization warps the image and breaks SLAM geometry: force it off where supported.
        val vs = chars.get(CameraCharacteristics.CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES)
        if (vs?.contains(CameraMetadata.CONTROL_VIDEO_STABILIZATION_MODE_OFF) == true)
            b.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CameraMetadata.CONTROL_VIDEO_STABILIZATION_MODE_OFF)
        val ois = chars.get(CameraCharacteristics.LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION)
        if (ois?.contains(CameraMetadata.LENS_OPTICAL_STABILIZATION_MODE_OFF) == true)
            b.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CameraMetadata.LENS_OPTICAL_STABILIZATION_MODE_OFF)
        val af = chars.get(CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES)
        if (af?.contains(CameraMetadata.CONTROL_AF_MODE_CONTINUOUS_VIDEO) == true)
            b.set(CaptureRequest.CONTROL_AF_MODE, CameraMetadata.CONTROL_AF_MODE_CONTINUOUS_VIDEO)
        return b
    }

    private fun releaseCodec() {
        try { codec?.stop() } catch (_: Exception) {}
        try { codec?.release() } catch (_: Exception) {}
        codec = null
        try { encSurface?.release() } catch (_: Exception) {}
        encSurface = null
    }

    private fun cleanup() {
        try { session?.stopRepeating() } catch (_: Exception) {}
        try { session?.close() } catch (_: Exception) {}
        session = null
        try { camera?.close() } catch (_: Exception) {}
        camera = null
        releaseCodec()
        try { prevSurface?.release() } catch (_: Exception) {}
        prevSurface = null
    }
}

/* ------------------------------------------------------------------------- */
/* Network: ONE writer thread, queue in front of it. Wire format unchanged:    */
/*   [u32 header_len][json header][payload of header.payload_len bytes]        */
/* ------------------------------------------------------------------------- */
class NavSenseClient(
    private val host: String,
    private val port: Int,
    private val onStatus: (String) -> Unit,
    private val onStart: () -> Unit,
    private val onLost: () -> Unit
) {
    @Volatile private var closed = false
    private var socket: Socket? = null
    private val queue = LinkedBlockingQueue<Any>()      // ByteArray (pre-framed JSON) or VideoPacket
    private val videoBacklog = AtomicLong(0)
    private var dropUntilKey = false
    val bytesSent = AtomicLong(0)
    @Volatile var dropped = 0L
        private set

    fun start() { Thread({ run() }, "net-writer").start() }

    fun sendJson(json: String) {
        if (closed) return
        val b = json.toByteArray(Charsets.UTF_8)
        val framed = ByteArray(4 + b.size)
        framed[0] = (b.size ushr 24).toByte(); framed[1] = (b.size ushr 16).toByte()
        framed[2] = (b.size ushr 8).toByte(); framed[3] = b.size.toByte()
        System.arraycopy(b, 0, framed, 4, b.size)
        queue.offer(framed)
    }

    /** Called from the encoder thread. IMU/control are never dropped; video is, but only cleanly. */
    fun sendVideo(p: VideoPacket) {
        if (closed) return
        if (!p.config) {
            if (dropUntilKey && !p.key) { dropped++; return }
            if (!p.key && videoBacklog.get() > Cfg.MAX_VIDEO_BACKLOG_BYTES) {
                dropUntilKey = true; dropped++; return
            }
            if (p.key) dropUntilKey = false
        }
        videoBacklog.addAndGet(p.data.size.toLong())
        queue.offer(p)
    }

    private fun run() {
        try {
            val s = Socket()
            s.tcpNoDelay = true
            s.connect(InetSocketAddress(host, port), 3000)
            socket = s
            val out = DataOutputStream(BufferedOutputStream(s.getOutputStream(), 256 * 1024))
            onStatus("Connected to $host:$port, syncing clocks...")
            sendJson("""{"type":"hello","version":2,"phone_wall_ms":${System.currentTimeMillis()},"phone_mono_ns":${SystemClock.elapsedRealtimeNanos()}}""")
            Thread({ readLoop(s) }, "net-reader").start()

            while (!closed) {
                val item = queue.poll(200, TimeUnit.MILLISECONDS) ?: continue
                when (item) {
                    is ByteArray -> { out.write(item); bytesSent.addAndGet(item.size.toLong()) }
                    is VideoPacket -> { videoBacklog.addAndGet(-item.data.size.toLong()); writeVideo(out, item) }
                }
                if (queue.isEmpty()) out.flush()   // coalesce small writes while there's a backlog
            }
        } catch (e: Exception) {
            if (!closed) onStatus("Network error: ${e.message}")
        } finally {
            close(lost = true)
        }
    }

    private fun writeVideo(out: DataOutputStream, p: VideoPacket) {
        val sendNs = SystemClock.elapsedRealtimeNanos()
        val meta = """{"type":"video","codec":"h264","frame_id":${p.frameId},"key":${p.key},"config":${p.config},"t_cam_ns":${p.tCamNs},"t_enc_ns":${p.tEncNs},"t_send_ns":$sendNs,"t_wall_ms":${System.currentTimeMillis()},"width":${p.width},"height":${p.height},"dropped":$dropped,"payload_len":${p.data.size}}"""
        val mb = meta.toByteArray(Charsets.UTF_8)
        out.writeInt(mb.size)
        out.write(mb)
        out.write(p.data)
        bytesSent.addAndGet((4 + mb.size + p.data.size).toLong())
    }

    private fun readLoop(s: Socket) {
        try {
            val input = DataInputStream(s.getInputStream().buffered(64 * 1024))
            while (!s.isClosed) {
                val n = input.readInt()
                if (n <= 0 || n > 1024 * 1024) throw IOException("invalid command size: $n")
                val data = ByteArray(n)
                input.readFully(data)
                val msg = String(data, Charsets.UTF_8)
                if (msg.contains("\"type\":\"sync_req\"")) {
                    val recvWall = System.currentTimeMillis()
                    val recvMono = SystemClock.elapsedRealtimeNanos()
                    sendJson("""{"type":"sync_resp","phone_recv_wall_ms":$recvWall,"phone_recv_mono_ns":$recvMono,"phone_send_wall_ms":${System.currentTimeMillis()},"phone_send_mono_ns":${SystemClock.elapsedRealtimeNanos()}}""")
                } else if (msg.contains("\"type\":\"start\"")) {
                    onStatus("Clocks synced, starting streams")
                    onStart()
                }
            }
        } catch (e: Exception) {
            if (!closed) { onStatus("Disconnected: ${e.message}"); close(lost = true) }
        }
    }

    fun close(lost: Boolean = false) {
        if (closed) return
        closed = true
        try { socket?.close() } catch (_: Exception) {}
        socket = null
        if (lost) onLost()
    }
}