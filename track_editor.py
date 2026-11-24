"""Draw the route the object should follow. Run: python track_editor.py

Left click = add waypoint | right click / u = undo | c = clear | l = loop on/off
a = add marker position | s = save | q / Esc = quit
"""
import cv2

import config as C
from track import Track, draw_track
from tracker import MarkerTracker, draw_detection

WIN = "Track editor"
HELP = "L-click add | R-click/u undo | c clear | l loop | a add marker pos | s save | q quit"


def main():
    tracker = MarkerTracker()
    if not tracker.open():
        print(f"ERROR: could not open camera {C.CAMERA_INDEX}. Close other apps using it and check CAMERA_INDEX in config.py.")
        return 1

    track = Track.load() or Track()
    state = {"msg": "", "frame_size": None}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            track.add(x, y)
        elif event == cv2.EVENT_RBUTTONDOWN:
            track.undo()

    cv2.namedWindow(WIN)
    cv2.setMouseCallback(WIN, on_mouse)
    try:
        while True:
            frame = tracker.read()
            if frame is None:
                if cv2.waitKey(10) & 0xFF in (ord("q"), 27):
                    break
                continue
            h, w = frame.shape[:2]
            track.frame_size = (w, h)
            det = tracker.detect(frame)
            draw_detection(frame, det)
            draw_track(frame, track)

            cv2.rectangle(frame, (0, 0), (w, 54), (0, 0, 0), -1)
            cv2.putText(frame, HELP, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            info = f"points: {len(track.points)}   mode: {'LOOP' if track.closed else 'START -> FINISH'}   {state['msg']}"
            cv2.putText(frame, info, (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(WIN, frame)

            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            elif k == ord("u"):
                track.undo()
            elif k == ord("c"):
                track.clear()
            elif k == ord("l"):
                track.closed = not track.closed
            elif k == ord("a"):
                if det.found:
                    track.add(*det.center)
                    state["msg"] = "added marker position"
                else:
                    state["msg"] = "marker not visible!"
            elif k == ord("s"):
                if len(track.points) < 2:
                    state["msg"] = "need at least 2 points"
                else:
                    track.save()
                    state["msg"] = f"saved to {C.TRACK_FILE}"
                    print(f"Saved {len(track.points)} points to {C.TRACK_FILE}")
    finally:
        tracker.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
