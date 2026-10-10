package com.navsense.logger

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbManager
import android.os.Build
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import androidx.core.content.ContextCompat
import com.hoho.android.usbserial.driver.UsbSerialDriver
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import com.hoho.android.usbserial.util.SerialInputOutputManager

/**
 * Phone -> Arduino bridge over USB-OTG. Accepts w/a/d (anything else = stop).
 * Laptop sends the current command as a heartbeat; if heartbeats stop, we send 's'.
 */
class CarLink(private val ctx: Context) {
    companion object {
        const val ACTION_USB_PERMISSION = "com.navsense.logger.USB_PERMISSION"
        const val WATCHDOG_MS = 600L      // raise if your Wi-Fi jitter makes the car stutter-stop
        const val BAUD = 9600             // must match Serial.begin() in the sketch
    }

    @Volatile var state = "connecting"
    @Volatile var rx = 0L                 // commands received from the laptop (diagnostic)
    @Volatile var lastRx = '-'
    @Volatile var tx = 0L                 // commands written to the Arduino
    @Volatile var heard = 0L              // bytes received back from the Arduino
    val summary: String get() = "$state | cmds=$rx '$lastRx' | wrote=$tx | heard=${heard}B"
    var onReply: ((String) -> Unit)? = null

    private val thread = HandlerThread("car").apply { start() }
    private val handler = Handler(thread.looper)      // all serial writes are serialized here
    private val um = ctx.getSystemService(Context.USB_SERVICE) as UsbManager
    private var port: UsbSerialPort? = null
    private var io: SerialInputOutputManager? = null
    private var last = 's'
    private var lastWriteMs = 0L
    private var receiverRegistered = false
    @Volatile private var closing = false

    private val permissionReceiver = object : BroadcastReceiver() {
        override fun onReceive(c: Context, i: Intent) {
            if (i.action != ACTION_USB_PERMISSION) return
            if (i.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) handler.post { openFirst() }
            else state = "USB permission denied"
        }
    }

    private fun findDriver(): UsbSerialDriver? =
        UsbSerialProber.getDefaultProber().findAllDrivers(um).firstOrNull()

    fun connect() {
        handler.post {
            val driver = findDriver()
            if (driver == null) {
                state = "no USB serial device (retrying)"
                if (!closing) handler.postDelayed({ connect() }, 2000)   // hub/board may enumerate late
                return@post
            }
            if (um.hasPermission(driver.device)) { openFirst(); return@post }
            state = "waiting for USB permission"
            ContextCompat.registerReceiver(
                ctx, permissionReceiver, IntentFilter(ACTION_USB_PERMISSION), ContextCompat.RECEIVER_NOT_EXPORTED
            )
            receiverRegistered = true
            // Android 14 needs an explicit intent + mutable flag for this PendingIntent.
            val flags = if (Build.VERSION.SDK_INT >= 31) PendingIntent.FLAG_MUTABLE else 0
            val pi = PendingIntent.getBroadcast(
                ctx, 0, Intent(ACTION_USB_PERMISSION).setPackage(ctx.packageName), flags
            )
            um.requestPermission(driver.device, pi)
        }
    }

    private fun openFirst() {
        try {
            val driver = findDriver() ?: run { state = "device gone"; return }
            val conn = um.openDevice(driver.device) ?: run { state = "openDevice failed"; return }
            val p = driver.ports[0]
            p.open(conn)
            p.setParameters(BAUD, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
            // Some boards (CDC-ACM/native USB) won't talk until DTR is asserted; Uno-types reset here.
            try { p.setDTR(true); p.setRTS(true) } catch (_: Exception) {}
            port = p
            // Keep draining the Arduino's replies so its TX never backs up; forward them to the laptop.
            io = SerialInputOutputManager(p, object : SerialInputOutputManager.Listener {
                override fun onNewData(data: ByteArray) { heard += data.size; onReply?.invoke(String(data)) }
                override fun onRunError(e: Exception) { state = "serial error: ${e.message}" }
            }).also { it.start() }
            last = 's'
            state = "booting"   // Uno/Nano reset when the port opens; ~2 s before they listen
            handler.postDelayed({ if (state == "booting") state = "ready" }, 2000)
        } catch (e: Exception) {
            state = "open failed: ${e.message}"
        }
    }

    /** Safe to call from any thread. */
    fun command(c: Char) {
        rx++; lastRx = c
        handler.post {
            val p = port ?: return@post
            val out = if (c == 'w' || c == 'a' || c == 'd') c else 's'
            val now = SystemClock.uptimeMillis()
            // Write on change, plus a 500 ms refresh for movement (covers lost writes and an Arduino dead-man timer).
            if (out != last || (out != 's' && now - lastWriteMs > 500)) {
                try {
                    p.write(byteArrayOf(out.code.toByte()), 200)
                    last = out; lastWriteMs = now; tx++
                } catch (e: Exception) {
                    state = "write error: ${e.message}"
                }
            }
            handler.removeCallbacks(watchdog)
            if (out != 's') handler.postDelayed(watchdog, WATCHDOG_MS)
        }
    }

    private val watchdog = Runnable {
        try {
            port?.write(byteArrayOf('s'.code.toByte()), 200)
            last = 's'
            state = "watchdog stop"
        } catch (e: Exception) {
            state = "write error: ${e.message}"
        }
    }

    fun close() {
        closing = true
        handler.post {
            handler.removeCallbacks(watchdog)
            try { port?.write(byteArrayOf('s'.code.toByte()), 200) } catch (_: Exception) {}
            try { io?.stop() } catch (_: Exception) {}
            try { port?.close() } catch (_: Exception) {}
            port = null; io = null
            if (receiverRegistered) {
                try { ctx.unregisterReceiver(permissionReceiver) } catch (_: Exception) {}
                receiverRegistered = false
            }
            thread.quitSafely()
        }
    }
}