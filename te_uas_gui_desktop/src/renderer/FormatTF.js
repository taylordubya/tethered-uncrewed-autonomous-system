import PropTypes from 'prop-types';
// Define FormatTF
export default function FormatTF({ trueText, falseText, value, hasText }) {
  let styleColor;
  let text;

  /* Format color and output text */
  if (value) {
    // True === Green
    styleColor = '#44FF34';
    // No input text defaults to TRUE output text
    if (trueText === '') {
      text = 'TRUE';
    } else {
      text = trueText;
    }
  } else {
    // False === Red
    styleColor = '#F00';
    // No input text defaults to FALSE output text
    if (falseText === '') {
      text = 'FALSE';
    } else {
      text = falseText;
    }
  }

  // Return just the color
  if (!hasText) {
    return styleColor;
  }
  return <strong style={{ color: styleColor }}>{text}</strong>;
}
FormatTF.defaultProps = {
  trueText: '',
  falseText: '',
};
FormatTF.propTypes = {
  trueText: PropTypes.string,
  falseText: PropTypes.string,
  value: PropTypes.bool.isRequired,
  hasText: PropTypes.bool.isRequired,
};
