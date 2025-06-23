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
  } catch (error) {
    console.error("Failed to load font", font, error);
  }
};

try {
  await Promise.all(fonts.map((font) => loadFont(font)));
} catch (error) {
  console.error("Critical font loading error", error);
}
