package org.maplibre.android.testapp.activity.style

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import org.maplibre.android.camera.CameraPosition
import org.maplibre.android.constants.MapLibreConstants
import org.maplibre.android.geometry.LatLng
import org.maplibre.android.maps.MapLibreMap
import org.maplibre.android.maps.MapView
import org.maplibre.android.maps.Style
import org.maplibre.android.testapp.R

/**
 * Isomaps : évaluation du terrain 3D avec le style de production (satellite +
 * DEM cdn.iso-maps.com, mode basemap direct du fork) — le MÊME JSON que l'app
 * d'éval iOS (platform/ios/app/isomaps_terrain_style.json), pour comparer le
 * rendu OpenGL Android au rendu Metal iOS à armes égales.
 */
class TerrainIsomapsActivity : AppCompatActivity() {
    private lateinit var mapView: MapView
    private lateinit var maplibreMap: MapLibreMap

    public override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_fill_extrusion_layer)
        mapView = findViewById(R.id.mapView)
        mapView.onCreate(savedInstanceState)
        mapView.getMapAsync { map ->
            maplibreMap = map
            // Laisser le geste d'inclinaison atteindre le pitch max supporté (85°)
            map.setMaxPitchPreference(MapLibreConstants.MAXIMUM_PITCH_LIMIT.toDouble())
            map.cameraPosition = CameraPosition.Builder()
                .target(LatLng(45.9237, 6.8694)) // Chamonix, face au massif du Mont-Blanc
                .zoom(11.5)
                .tilt(60.0)
                .bearing(150.0)
                .build()
            val json = assets.open("isomaps_terrain_style.json").bufferedReader().use { it.readText() }
            map.setStyle(Style.Builder().fromJson(json))
        }
    }

    override fun onStart() {
        super.onStart()
        mapView.onStart()
    }

    override fun onResume() {
        super.onResume()
        mapView.onResume()
    }

    override fun onPause() {
        super.onPause()
        mapView.onPause()
    }

    override fun onStop() {
        super.onStop()
        mapView.onStop()
    }

    public override fun onSaveInstanceState(outState: Bundle) {
        super.onSaveInstanceState(outState)
        mapView.onSaveInstanceState(outState)
    }

    override fun onLowMemory() {
        super.onLowMemory()
        mapView.onLowMemory()
    }

    public override fun onDestroy() {
        super.onDestroy()
        mapView.onDestroy()
    }
}
