PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//247135/1238926/2.50/11/3/Connector

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "r138_45"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 0.450) (shapeHeight 1.380))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(padStyleDef "c214.5_h143"
		(holeDiam 1.43)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 2.145) (shapeHeight 2.145))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 2.145) (shapeHeight 2.145))
	)
	(padStyleDef "r155_142.5"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 1.425) (shapeHeight 1.550))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(padStyleDef "r190_87.5"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 0.875) (shapeHeight 1.900))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "1051640001" (originalName "1051640001")
		(multiLayer
			(pad (padNum 1) (padStyleRef r138_45) (pt -1.300, -2.310) (rotation 0))
			(pad (padNum 2) (padStyleRef r138_45) (pt -0.650, -2.310) (rotation 0))
			(pad (padNum 3) (padStyleRef r138_45) (pt 0.000, -2.310) (rotation 0))
			(pad (padNum 4) (padStyleRef r138_45) (pt 0.650, -2.310) (rotation 0))
			(pad (padNum 5) (padStyleRef r138_45) (pt 1.300, -2.310) (rotation 0))
			(pad (padNum 6) (padStyleRef c214.5_h143) (pt -3.950, 0.350) (rotation 90))
			(pad (padNum 7) (padStyleRef c214.5_h143) (pt 3.950, 0.350) (rotation 90))
			(pad (padNum 8) (padStyleRef r155_142.5) (pt -2.487, -2.225) (rotation 0))
			(pad (padNum 9) (padStyleRef r155_142.5) (pt 2.487, -2.225) (rotation 0))
			(pad (padNum 10) (padStyleRef r190_87.5) (pt -1.188, 0.350) (rotation 0))
			(pad (padNum 11) (padStyleRef r190_87.5) (pt 1.188, 0.350) (rotation 0))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0.000, -0.250) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef 28)
			(line (pt -3.75 2.5) (pt 3.75 2.5) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 3.75 2.5) (pt 3.75 -2.5) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 3.75 -2.5) (pt -3.75 -2.5) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -3.75 -2.5) (pt -3.75 2.5) (width 0.025))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -6.022 -4) (pt 6.022 -4) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 6.022 -4) (pt 6.022 3.5) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 6.022 3.5) (pt -6.022 3.5) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -6.022 3.5) (pt -6.022 -4) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -3.4 -2.5) (pt -3.75 -2.5) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -3.75 -2.5) (pt -3.75 -1) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -3.75 1.7) (pt -3.75 2.5) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -3.75 2.5) (pt 3.75 2.5) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 3.75 2.5) (pt 3.75 1.7) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 3.75 -1) (pt 3.75 -2.5) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 3.75 -2.5) (pt 3.4 -2.5) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.3 -3.5) (pt -1.3 -3.5) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -1.3, -3.55) (radius 0.05) (startAngle 90.0) (sweepAngle 180.0) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.3 -3.6) (pt -1.3 -3.6) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(arc (pt -1.3, -3.55) (radius 0.05) (startAngle 270) (sweepAngle 180.0) (width 0.1))
		)
	)
	(symbolDef "105164-0001" (originalName "105164-0001")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 3) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 4) (pt 0 mils -300 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -325 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 5) (pt 0 mils -400 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -425 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 6) (pt 0 mils -500 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -525 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 7) (pt 1100 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 8) (pt 1100 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 9) (pt 1100 mils -200 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -225 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 10) (pt 1100 mils -300 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -325 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(pin (pinNum 11) (pt 1100 mils -400 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -425 mils) (rotation 0]) (justify "Right") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 100 mils) (pt 900 mils 100 mils) (width 6 mils))
		(line (pt 900 mils 100 mils) (pt 900 mils -600 mils) (width 6 mils))
		(line (pt 900 mils -600 mils) (pt 200 mils -600 mils) (width 6 mils))
		(line (pt 200 mils -600 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 950 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 950 mils 200 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "105164-0001" (originalName "105164-0001") (compHeader (numPins 11) (numParts 1) (refDesPrefix J)
		)
		(compPin "1" (pinName "VBUS") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "2" (pinName "D-") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "3" (pinName "D+") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "4" (pinName "ID") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "5" (pinName "GND") (partNum 1) (symPinNum 5) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "MH1" (pinName "MH1") (partNum 1) (symPinNum 6) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "MH2" (pinName "MH2") (partNum 1) (symPinNum 7) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "MP1" (pinName "MP1") (partNum 1) (symPinNum 8) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "MP2" (pinName "MP2") (partNum 1) (symPinNum 9) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "MP3" (pinName "MP3") (partNum 1) (symPinNum 10) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "MP4" (pinName "MP4") (partNum 1) (symPinNum 11) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "105164-0001"))
		(attachedPattern (patternNum 1) (patternName "1051640001")
			(numPads 11)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
				(padNum 5) (compPinRef "5")
				(padNum 6) (compPinRef "MH1")
				(padNum 7) (compPinRef "MH2")
				(padNum 8) (compPinRef "MP1")
				(padNum 9) (compPinRef "MP2")
				(padNum 10) (compPinRef "MP3")
				(padNum 11) (compPinRef "MP4")
			)
		)
		(attr "Mouser Part Number" "538-105164-0001")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/Molex/105164-0001?qs=JAwGc6FMfvtRY4SkW%2FoGcQ%3D%3D")
		(attr "Manufacturer_Name" "Molex")
		(attr "Manufacturer_Part_Number" "105164-0001")
		(attr "Description" "USB - micro B USB 2.0 Receptacle Connector 5 Position Surface Mount, Right Angle; Through Hole")
		(attr "<Hyperlink>" "https://www.molex.com/pdm_docs/sd/1051640001_sd.pdf")
		(attr "<Component Height>" "2.7")
		(attr "<STEP Filename>" "105164-0001.stp")
		(attr "<STEP Offsets>" "X=0;Y=-1.17;Z=0.41")
		(attr "<STEP Rotation>" "X=90;Y=0;Z=0")
	)

)