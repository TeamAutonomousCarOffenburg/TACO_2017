from PyQt4.QtGui import QPen
from PyQt4.Qt import Qt
from sloth.items import RectItem

#from sloth.conf.default_config import HOTKEYS


class CustomAADC(RectItem):
    # display values of x and y as text inside the rectangle
    #defaultAutoTextKeys = ['x', 'y']

    def __init__(self, *args, **kwargs):
        RectItem.__init__(self, *args, **kwargs)

        # set drawing pen to red with width 2
        self.setPen(QPen(Qt.red, 2))


LABELS = ({
    "item": CustomAADC,
    "inserter": "sloth.items.RectItemInserter",
    "attributes": {
        "class": "stop_line_left"
    },
    "text": "stop_line_left",
    "hotkey": "1"
}, {
    "item": CustomAADC,
    "inserter": "sloth.items.RectItemInserter",
    "attributes": {
        "class": "stop_line_right"
    },
    "text": "stop_line_right",
    "hotkey": "2"
}, {
    "item": CustomAADC,
    "inserter": "sloth.items.RectItemInserter",
    "attributes": {
        "class": "stop_line_ahead"
    },
    "text": "stop_line_ahead",
    "hotkey": "3"
}, {
    "item": CustomAADC,
    "inserter": "sloth.items.RectItemInserter",
    "attributes": {
        "class": "stop_line_oncoming"
    },
    "text": "stop_line_oncoming",
    "hotkey": "4"
}, {
    "item": CustomAADC,
    "inserter": "sloth.items.RectItemInserter",
    "attributes": {
        "class": "crosswalk"
    },
    "text": "crosswalk",
    "hotkey": "5"
}, {
    "item": CustomAADC,
    "inserter": "sloth.items.RectItemInserter",
    "attributes": {
        "class": "person"
    },
    "text": "person",
    "hotkey": "f"
}, {
    "item": CustomAADC,
    "inserter": "sloth.items.RectItemInserter",
    "attributes": {
        "class": "child"
    },
    "text": "child",
    "hotkey": "c"
}, {
    "item": CustomAADC,
    "inserter": "sloth.items.RectItemInserter",
    "attributes": {
        "class": "middle_lane"
    },
    "text": "middle_lane",
    "hotkey": "g"
}, {
    "item": CustomAADC,
    "inserter": "sloth.items.RectItemInserter",
    "attributes": {
        "class": "car"
    },
    "text": "car",
    "hotkey": "a"
}, {
    "item": CustomAADC,
    "inserter": "sloth.items.RectItemInserter",
    "attributes": {
        "class": "person_crossing"
    },
    "text": "person_crossing",
    "hotkey": "x"
})
