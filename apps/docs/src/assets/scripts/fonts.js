const fonts = [
  {
    fontFamily: "slkscr",
    url: "url('/fonts/slkscr.ttf') format('truetype')",
  },
];

const loadFont = async (font) => {
  try {
    const fontFace = new FontFace(font.fontFamily, font.url);
    await fontFace.load();
    if (!document.fonts.has(fontFace)) {
      document.fonts.add(fontFace);
    }
  } catch (err) {
    console.error("Failed to load font", font, err);
  }
};

const loadFonts = async () => {
  try {
    await Promise.all(fonts.map(loadFont));
  } catch (err) {
    console.error("Critical font loading error", err);
  }
};

loadFonts();
