// ToolDrawing.h : the Tool dialog's drawing of what each measurement means.

#pragma once

#include "ToolLibrary.h"

// Draws the tool, tip down, to its own proportions (the cutting and shank
// diameters, the flute and overall lengths, the corner radius, tip diameter,
// angles, pitch and teeth as typed; missing ones guessed), filling rect. Each
// measurement carries the letter on its field's label in the Tool dialog:
// a diameter, b flute length, c overall length, d shank length, e shank diameter,
// f corner radius, g tip diameter, h included angle, i taper angle, j point angle,
// k pitch, l thread angle. The dimensions and letters keep their own size.
void DrawToolDiagram(HDC hdc, const RECT& rect, const camcore::Tool& tool);
