function updateFavicon() {
    const favicon = document.querySelector("link[rel='icon']");
    const logo = document.querySelector("[data-md-component='logo'] img");
    if (!favicon || !logo) return;

    favicon.href = logo.src.replace(/(?:-dark)?\.svg$/, `${document.body.dataset.mdColorScheme === "slate" ? "-dark" : ""
        }.svg`);
}

document$.subscribe(updateFavicon);
new MutationObserver(updateFavicon).observe(document.body, {
    attributes: true,
    attributeFilter: ["data-md-color-scheme"],
});
