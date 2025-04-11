import "../Home.css"

export default function StatusWidget({
    name,
    status,
    imgSrc,
    altSrc,
}) {
    return (
        <div id="environmnet-info">
            <div id="info-title">{name}</div>
            <img src={imgSrc} alt={altSrc} />
            <div id="info-status">{status}</div>
        </div>
    )
}