import ogImageSrc from "@images/logo.png";

export const SITE = {
  title: "seed",
  description: "zzk13180 seed",
  url: "https://github.com/zzk13180/seed",
  author: "seed",
};

export const SEO = {
  title: SITE.title,
  description: SITE.description,
  structuredData: {
    "@context": "https://github.com/zzk13180/seed",
    "@type": "WebPage",
    inLanguage: "zh",
    "@id": SITE.url,
    url: SITE.url,
    name: SITE.title,
    description: SITE.description,
    isPartOf: {
      "@type": "WebSite",
      url: SITE.url,
      name: SITE.title,
      description: SITE.description,
    },
  },
};

export const OG = {
  locale: "zh",
  type: "website",
  url: SITE.url,
  title: `${SITE.title}: : zzk13180,seed`,
  description: "zzk13180,seed",
  image: ogImageSrc,
};
